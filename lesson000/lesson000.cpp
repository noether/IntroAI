#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>

#include "agent.hpp"

int main()
{
    // Limits of the rendering window
    int wwidth = 800;
    int wheight = 800;
    sf::RenderWindow window(sf::VideoMode(wwidth, wheight), "Lesson 0");

    // Time will be measured in microseconds
    sf::Clock clock;
    sf::Time elapsed;
    int dt = 1e4; // 0.01 secs

    // We set an arbirtary number of agents in the simulation
    // The color of the first agent will be red, the last one green, the 
    // rest by default white.
    // The initial velocities are chosen randomly.
    int num_agents = 5;
    std::vector<Agent> agents;

    for(int i = 0; i < num_agents; i++){

        Eigen::Vector2f init_pos = wwidth/2*Eigen::Vector2f::Ones()
            + wwidth/2*Eigen::Vector2f::Random();
        Eigen::Vector2f init_vel = Eigen::Vector2f::Random();
        init_vel.normalize();

        Agent agent(i, init_pos, 30*init_vel);
        agent.setIntegrationTime(dt*1e-6); // In seconds
        agent.setPositionLimits(wwidth, wheight);
        if(i == 0)
            agent.setColorShape(sf::Color::Red);
        else if(i == num_agents-1)
            agent.setColorShape(sf::Color::Green);

        agents.push_back(agent);
    }


    // Main loop
    // The time interval between two iterations is dt microseconds.
    while(window.isOpen())
    {
        // We start the clock in order to measure the time consumed
        // by one iteration.
        clock.restart();

        // If you close the rendering window, we exit from the main loop
        // and the program is over.
        sf::Event event;
        while (window.pollEvent(event)){
            if (event.type == sf::Event::Closed)
                window.close();
        }

        // Clear the rendering window before drawing on it.
        window.clear(sf::Color::Black);

        // We consider the first agent as an unicycle with constant
        // speed and constant angular velocity (red triangle).
        //
        // We consider the last agent with 1st order dynamics with
        // a constant velocity (10, 10) (green circle).
        //
        // We consider the rest of agents with 2nd order dynamics with
        // zero acceleration, therefore their velocities are given
        // by the initial random velocities (white circles).
        for (std::vector<Agent>::iterator it = agents.begin();
            it != agents.end(); ++it){

            if(it == agents.begin()){
                it->updateUnicycle(0, 30*M_PI/180); // 30 degrees/sec
                it->draw(&window, Shape::triangle);
            }else if(it == agents.end()-1)
            {
                it->update1stDyn(10*Eigen::Vector2f::Ones());
                it->draw(&window, Shape::circle);
            }else
            {
                it->update2ndDyn(Eigen::Vector2f::Zero());
                it->draw(&window, Shape::circle);
            }
        }

        // Draw on the screen
        window.display();
        
        // Wait until 'dt' misconseconds are consumed
        sf::sleep(sf::microseconds
                (dt - clock.getElapsedTime().asMicroseconds()));
    }

    return 0;
}
