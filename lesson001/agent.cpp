#include <math.h>
#include <SFML/Graphics.hpp>
#include <Eigen/Core>

#include "agent.hpp"

#include <iostream>

// Helping function, the heading angle is always between -PI and PI
float norm_theta(float theta)
{
    while(abs(theta) > M_PI){
    if (theta > M_PI)
        theta -= 2*M_PI;
    else if(theta <= -M_PI)
        theta += 2*M_PI;
    }

    return theta;
}

Agent::Agent(int t, Eigen::Vector2f p, Eigen::Vector2f v):
    tag_(t),
    position_(p),
    velocity_(v)
{
    speed_ = v.norm();
    if (speed_ < 1e-5){
        speed_ = 0;
        theta_ = 0;
    }else
    {
        theta_ = atan2(v(1), v(0));
    }

    // White color by default for the triangle or circle
    setColorShape(sf::Color::White);

    // Setting an isosceles triangle
    isotriang_.setPointCount(3);
    isotriang_.setPoint(0, sf::Vector2f(-5, -7));
    isotriang_.setPoint(1, sf::Vector2f(0, 13));
    isotriang_.setPoint(2, sf::Vector2f(5, -7));
    isotriang_.setFillColor(color_shape_);

    // Setting a circle
    circle_.setRadius(5);
    circle_.setFillColor(color_shape_);
    circle_.setOrigin(5, 5);

    // The origin of the World Coordinates is at the bottom-left corner of the 
    // rendering window. X-axis positive to the right,
    // Y-axis possitive is upwards.
    //
    // Graphics are in the usual UV Coordinates, whose origin is at the
    // up-left corner and X-axis positive to the right and Y-axis downwards.
    //
    // All the calculations are in World Coordinates, so we need a 
    // coordinate transformation to UV Coordinates for drawing.
    setPositionGraphics_();
    setOrientationGraphics_();

    // Default settings for the trajectory described by the agent
    setTrajectorySize(200);
    setDrawTrajectory(true);
    setColorTrajectory(sf::Color::Blue);
}

Agent::Agent() : Agent(0, Eigen::Vector2f::Zero(), Eigen::Vector2f::Zero())
{
    theta_ = 0;
}

void Agent::setIntegrationTime(float dt)
{
    dt_ = dt;
}

void Agent::setPositionLimits(int w, int h)
{
    limitX_ = w;
    limitY_ = h;
}

void Agent::setPosition(Eigen::Vector2f p)
{
    position_ = p;
}

void Agent::setColorShape(sf::Color color)
{
    color_shape_ = color;
    isotriang_.setFillColor(color_shape_);
    circle_.setFillColor(color_shape_);
}

void Agent::setColorTrajectory(sf::Color color)
{
    color_trajectory_ = color;
}

void Agent::setTrajectorySize(unsigned int s)
{
    trajectory_max_size_ = s;
}

void Agent::setDrawTrajectory(bool s)
{
    draw_trajectory_ = s;
}

void Agent::setPositionGraphics_()
{
    circle_.setPosition(position_(0), -position_(1) + limitY_);
    isotriang_.setPosition(position_(0), -position_(1) + limitY_);

    // We keep a history of the last agent's positions
    trajectory_.push_back(position_);
    if(trajectory_.size() > trajectory_max_size_)
        trajectory_.erase(trajectory_.begin());
}

void Agent::setOrientationGraphics_()
{
    isotriang_.setRotation(-theta_*180/M_PI - 90);
}

void Agent::setVelocity(Eigen::Vector2f v)
{
    velocity_ = v;
}

Eigen::Vector2f Agent::getPosition()
{
    return position_;
}

Eigen::Vector2f Agent::getVelocity()
{
    return velocity_;
}

void Agent::update1stDyn(Eigen::Vector2f u)
{
    // Update of World Coordinates
    velocity_ = u;
    position_ += u*dt_;

    // For really small velocities we do not update theta
    // In fact, theta should not be useful for 1st order dynamics
    // but for being consistent with the code.
    speed_ = velocity_.norm();
    if (speed_ > 1e-5)
        theta_ = atan2(velocity_(1), velocity_(0));

    // Update of UV Coordinates
    setPositionGraphics_();
    setOrientationGraphics_();
}

void Agent::update2ndDyn(Eigen::Vector2f u)
{
    // Update of World Coordinates
    velocity_ += u*dt_;
    position_ += velocity_*dt_;

    // For really small velocities we do not update theta
    // In fact, theta should not be useful for 2nd order dynamics
    // but for being consistent with the code.
    speed_ = velocity_.norm();
    if (speed_ > 1e-5)
        theta_ = atan2(velocity_(1), velocity_(0));

    // Update of UV Coordinates
    setPositionGraphics_();
    setOrientationGraphics_();
}

void Agent::updateUnicycle(float us, float ut)
{
    speed_ += us*dt_;
    theta_ += ut*dt_;

    // We do not allow to go backwards. In fact we usually assume
    // that the speed is constant so 'us = 0' most of the cases.
    if (speed_ < 0)
        speed_ = 0;

    theta_ = norm_theta(theta_);

    // Update of World Coordinates
    velocity_(0) = speed_*cos(theta_);
    velocity_(1) = speed_*sin(theta_);
    position_ += velocity_*dt_;

    // Update of UV Coordinates
    setPositionGraphics_();
    setOrientationGraphics_();
}

void Agent::draw(sf::RenderWindow *w, Shape s)
{
    // Draw a isosceles triangle or a circle.
    switch(s){
        case Shape::triangle:
            w->draw(isotriang_);
            break;
        case Shape::circle:
            w->draw(circle_);
            break;
        default:
            break;
    }

    // We only draw the trajectory points within the rendering window.
    // 1. We make an image completely transparent.
    // 2. We draw the pixels of the trajectory on the image.
    // 3. We create a texture from such a image.
    // 4. We create a sprite from the texture.
    // 5. We draw the sprite.
    if(draw_trajectory_){
        sf::Image imagetraj;
        imagetraj.create(limitX_, limitY_, sf::Color::Black);
        imagetraj.createMaskFromColor(sf::Color::Black, 0);

        for (auto it = trajectory_.begin(); it != trajectory_.end(); ++it)
        {
            Eigen::Vector2f pos = *it;
        
            if(pos(0) <= limitX_ && pos(1) <= limitY_ &&
                    pos(0) > 0 && pos(1) > 0)
                imagetraj.setPixel(pos(0), -pos(1)+limitY_, color_trajectory_);
        }

        sf::Texture texture;
        texture.create(limitX_, limitY_);
        texture.update(imagetraj);

        sf::Sprite sprite;
        sprite.setTexture(texture);

        w->draw(sprite);
    }
}

void Agent::flocking(std::vector<Agent> *agents, float radius, 
        float kva, float ks, float kc, float ke)
{
    int neighbor_count = 0;
    Eigen::Vector2f velAvg = Eigen::Vector2f::Zero();
    Eigen::Vector2f centroid = Eigen::Vector2f::Zero();
    Eigen::Vector2f separation = Eigen::Vector2f::Zero();
    Eigen::Vector2f desired_velocity = Eigen::Vector2f::Zero();


    // We check all the agents on the screen.
    // Any agent closer than radius units is a neighbor.
    for (std::vector<Agent>::iterator it = agents->begin(); 
            it != agents->end(); ++it){

        Eigen::Vector2f neighbor = it->getPosition();
        Eigen::Vector2f relativePosition = neighbor - position_;

        if(relativePosition.norm() < radius){
            // We have found a neighbor
            neighbor_count++;

            // We add all the positions
            centroid += it->getPosition();

            // We add all the velocities
            velAvg += it->getVelocity();

            // Vector pointing at the opposite direction w.r.t. your 
            // neighbor
            separation -= relativePosition;
        }
    }

    centroid /= neighbor_count; // All the positions over the num of neighbors
    velAvg /= neighbor_count; // All the velocities over the numb of neighbors

    // Relative position of the agent w.r.t. centroid
    Eigen::Vector2f cohesion = centroid - position_;

    // In order to compare the following vectors we normalize all of them,
    // so they have the same magnitude. Later on with the gains 
    // kva, ks and kc we assing which vectors are more important.
    velAvg.normalize();
    cohesion.normalize();
    separation.normalize();

    if(neighbor_count == 1)
        desired_velocity = velocity_;
    else
        desired_velocity += kva*velAvg + ks*separation + kc*cohesion;

    float error_theta = atan2(desired_velocity(1), desired_velocity(0))
                        - theta_;

    updateUnicycle(0, ke*error_theta);

    if(position_(0) < 0 || position_(0) > limitX_
            || position_(1) < 0 || position_(1) > limitY_)
    {
        position_ = limitX_ / 2*Eigen::Vector2f::Ones() + 
            limitX_ / 2*Eigen::Vector2f::Random();
        velocity_ = Eigen::Vector2f::Random();
        theta_ = atan2(velocity_(1), velocity_(0));
    }
}


