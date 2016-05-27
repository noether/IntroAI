#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>

#include "agent.hpp"

int main()
{
    // Limits of the rendering window
    int wwidth = 800;
    int wheight = 800;
    sf::RenderWindow window(sf::VideoMode(wwidth, wheight), "Lesson 1 Flocking");

    // Time will be measured in microseconds
    sf::Clock clock;
    sf::Time elapsed;
    int dt = 1e4; // 0.01 secs

    // We set an arbirtary number of agents in the simulation
    // The color of the first agent will be red, the last one green, the 
    // rest by default white.
    // The initial velocities are chosen randomly.
    int num_agents = 60;
    std::vector<Agent> agents;

    for(int i = 0; i < num_agents; i++){

        Eigen::Vector2f init_pos = wwidth/2*Eigen::Vector2f::Ones()
            + wwidth/2*Eigen::Vector2f::Random();
        Eigen::Vector2f init_vel = Eigen::Vector2f::Random();
        init_vel.normalize();

        Agent agent(i, init_pos, (rand()%30 + 20)*init_vel);
        agent.setIntegrationTime(dt*1e-6); // In seconds
        agent.setPositionLimits(wwidth, wheight);
        agent.setDrawTrajectory(false);
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

        // Pass to each agent the vector of agents in order to 
        // run the flocking algorithm.
        // We apply to each gain a random factor between 0.9 and 1.2
        for (std::vector<Agent>::iterator it = agents.begin(); 
            it != agents.end(); ++it){

            it->flocking(&agents, 40,
                    0.3*(rand()%12 + 9)/10,
                    1*(rand()%12 + 9)/10,
                    1*(rand()%12 + 9)/10,
                    1.3*(rand()%12 + 9)/10);

            it->draw(&window, Shape::triangle);
        }


        // Draw on the screen
        window.display();
        
        // Wait until 'dt' misconseconds are consumed
        sf::sleep(sf::microseconds
                (dt - clock.getElapsedTime().asMicroseconds()));
    }

    return 0;
}
