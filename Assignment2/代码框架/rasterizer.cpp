// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(int x, int y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    //三条边的向量
    Eigen::Vector3f v0 = _v[0] - _v[1];
    Eigen::Vector3f v1 = _v[1] - _v[2];
    Eigen::Vector3f v2 = _v[2] - _v[0];
    
    //点与三条边组成的向量
    Eigen::Vector3f v0p = _v[0] - Eigen::Vector3f(x,y,0);
    Eigen::Vector3f v1p = _v[1] - Eigen::Vector3f(x,y,0);
    Eigen::Vector3f v2p = _v[2] - Eigen::Vector3f(x,y,0);

    float z1 = v0p.x()*v0.y() - v0p.y()*v0.x();
    float z2 = v1p.x()*v1.y() - v1p.y()*v1.x();
    float z3 = v2p.x()*v2.y() - v2p.y()*v2.x();

    if((z1>0 && z2>0 && z3>0) || (z1<0 && z2<0 && z3<0)){
        return true;
    }else {
        return false;
    }

}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    float x_min = width;
    float x_max = 0;
    float y_min = height;
    float y_max = 0;
    for(int i = 0; i<3; i++){
        x_min = std::min(x_min, v[i].x());
        x_max = std::max(x_max, v[i].x());
        y_min = std::min(y_min, v[i].y());
        y_max = std::max(y_max, v[i].y());
    }
    
    // std::cout<<width<<" " <<height<<std::endl;
    // std::cout << x_min << " " << x_max << " " << y_min << " " << y_max << std::endl;
    
    // iterate through the pixel and 
    for(int y = y_min; y < y_max; y++)
        for(int x = x_min; x < x_max; x++){
            float count = 0.0;
            float max_count = ssaa_w * ssaa_h;
            for(float i = start_point; i<1.0; i+=pixel_per_sm)
                for(float j = start_point; j<1.0; j+=pixel_per_sm){
                    if(insideTriangle(x+i,y+j,t.v)){
                        count+=1.0;
                    }
                }
                int i=0.5;
                int j=0.5;
                int index = get_index(x,y);
                //find if the current pixel is inside the triangle
                    if(insideTriangle(x+i,y+j,t.v)){
                        // If so, use the following code to get the interpolated z value.
                        auto[alpha, beta, gamma] = computeBarycentric2D(x+i,y+j,t.v);
                        float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;
                        // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
                        // int index = get_index_ssaa(x,y, i, j);
                        
                        if(z_interpolated < depth_buf[index]) {
                            Eigen::Vector3f p;
                            p << (float)x, (float)y, z_interpolated;
                            Eigen::Vector3f color = t.getColor()*(count/max_count)+((max_count-count)/max_count)*frame_buf[index];
                            set_pixel(p, color);
                            depth_buf[index] = z_interpolated;
                            
                            // depth_buf_ssaa[index] = z_interpolated;
                            // frame_buf_ssaa[index] = t.getColor();
                        }
                    }
                    if(count < max_count){
                        depth_buf[index] = std::numeric_limits<float>::infinity();
                    }
        }

}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::fill(frame_buf_ssaa.begin(), frame_buf_ssaa.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(depth_buf_ssaa.begin(), depth_buf_ssaa.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    //SSAA
    // frame_buf_ssaa.resize(w * h * ssaa_w * ssaa_h);
    // depth_buf_ssaa.resize(w * h * ssaa_w * ssaa_h);

}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

int rst::rasterizer::get_index_ssaa(int x, int y, float i, float j) {
    
    i = int((i-start_point)/pixel_per_sm);
    j = int((j-start_point)/pixel_per_sm);
    return (height*ssaa_h - 1 - y * ssaa_h + j)*width*ssaa_w + x*ssaa_w + i;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    // std::cout<<ind<<std::endl;
    frame_buf[ind] = color;

}

// clang-format on