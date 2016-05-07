#include <SFML/Graphics.hpp>
#include <Eigen/Core>
#include <vector>

enum Shape{triangle, circle};

class Agent{
    private:
        int tag_; // name of the agent

        // Position and Velocity are w.r.t. World Coordinates, whose
        // origin is at the bottom-left corner of the rendering window and
        // X-axis positive is to the right and Y-axis positive is upwards.
        // 
        // position [pixels]
        // velocity [pixels/time]
        Eigen::Vector2f position_, velocity_;

        // Norm of the velocity and heading angle.
        // We assume that the heading angle is always
        // the direction of the velocity.
        //
        // speed [pixels/time]
        // theta [rads]
        float speed_, theta_;

        float dt_; // simulation fixed time step

        // Graphics
        int limitX_, limitY_; // limits of the window for rendering

        // We can choose to represent the agent among an isosceles triangle
        // or a circle
        sf::ConvexShape isotriang_;
        sf::CircleShape circle_;
        sf::Color color_shape_;

        std::vector<Eigen::Vector2f> trajectory_;
        unsigned int trajectory_max_size_; // max num of points to be drawn
        bool draw_trajectory_;
        sf::Color color_trajectory_;

        void setPositionGraphics_(); // Shapes are in different coordinates
        void setOrientationGraphics_();

    public:
        Agent(int, Eigen::Vector2f, Eigen::Vector2f);
        Agent();

        void setIntegrationTime(float);
        void setPositionLimits(int, int);

        void setPosition(Eigen::Vector2f);
        void setVelocity(Eigen::Vector2f);
        Eigen::Vector2f getPosition();
        Eigen::Vector2f getVelocity();

        // For the agent's dynamics we can chose among three different models
        void update1stDyn(Eigen::Vector2f); // kinematical point
        void update2ndDyn(Eigen::Vector2f); // Newtonian point
        void updateUnicycle(float, float);  // Unicycle

        // Graphics
        void draw(sf::RenderWindow *, Shape s);
        void setColorShape(sf::Color);
        void setTrajectorySize(unsigned int);
        void setDrawTrajectory(bool);
        void setColorTrajectory(sf::Color);
};

