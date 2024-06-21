/**
 * SDL window creation adapted from https://github.com/isJuhn/DoublePendulum
*/
#include "simulate.h"

Eigen::MatrixXf LQR(PlanarQuadrotor &quadrotor, float dt) {
    /* Calculate LQR gain matrix */
    Eigen::MatrixXf Eye = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf A_discrete = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf B(6, 2);
    Eigen::MatrixXf B_discrete(6, 2);
    Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf R = Eigen::MatrixXf::Identity(2, 2);
    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(6, 6);
    Eigen::Vector2f input = quadrotor.GravityCompInput();

    Q.diagonal() << 100, 100, 400, 20, 10, 250 / 2 / M_PI;
    R.row(0) << 90, 0.05;
    R.row(1) << 0.05, 90;
    std::tie(A, B) = quadrotor.Linearize();
    A_discrete = Eye + dt * A;
    B_discrete = dt * B;
    
    return LQR(A_discrete, B_discrete, Q, R);
}

void control(PlanarQuadrotor &quadrotor, const Eigen::MatrixXf &K) {
    Eigen::Vector2f input = quadrotor.GravityCompInput();
    quadrotor.SetInput(input - K * quadrotor.GetControlState());
}

int main(int argc, char* args[])
{
    std::shared_ptr<SDL_Window> gWindow = nullptr;
    std::shared_ptr<SDL_Renderer> gRenderer = nullptr;
    const int SCREEN_WIDTH = 1280;
    const int SCREEN_HEIGHT = 720;

    /**
     * TODO: Extend simulation
     * 1. Set goal state of the mouse when clicking left mouse button (transform the coordinates to the quadrotor world! see visualizer TODO list)
     *    [x, y, 0, 0, 0, 0]
     * 2. Update PlanarQuadrotor from simulation when goal is changed
    */
    Eigen::VectorXf initial_state = Eigen::VectorXf::Zero(6);

    initial_state << SCREEN_WIDTH/2/128.0f,SCREEN_HEIGHT/2/128.0f,0,0,0,0;

    PlanarQuadrotor quadrotor(initial_state);
    PlanarQuadrotorVisualizer quadrotor_visualizer(&quadrotor);
    /**
     * Goal pose for the quadrotor
     * [x, y, theta, x_dot, y_dot, theta_dot]
     * For implemented LQR controller, it has to be [x, y, 0, 0, 0, 0]
    */
    Eigen::VectorXf goal_state = Eigen::VectorXf::Zero(6);
    goal_state << SCREEN_WIDTH/2/128.0f,SCREEN_HEIGHT/2/128.0f,0,0,0,0;
    quadrotor.SetGoal(goal_state);
    /* Timestep for the simulation */
    const float dt = 0.001;

    Eigen::MatrixXf K = LQR(quadrotor, dt);
    Eigen::Vector2f input = Eigen::Vector2f::Zero(2);

    /**
     * TODO: Plot x, y, theta over time
     * 1. Update x, y, theta history vectors to store trajectory of the quadrotor
     * 2. Plot trajectory using matplot++ when key 'p' is clicked
    */
    std::vector<float> x_history;
    std::vector<float> y_history;
    std::vector<float> theta_history;
    std::vector<float> time_history;

    x_history.push_back(quadrotor.GetState()[0]);
    y_history.push_back(quadrotor.GetState()[1]);
    theta_history.push_back(quadrotor.GetState()[2]);
    time_history.push_back(0);

    if (init(gWindow, gRenderer, SCREEN_WIDTH, SCREEN_HEIGHT) >= 0)
    {
        SDL_Event e;
        bool quit = false;
        float delay;
        int x, y;
        int animationCounter = 0;
        int vectorsCounter = 0;
        float x_div, y_div;

        Eigen::VectorXf state = Eigen::VectorXf::Zero(6);

        while (!quit)
        {
            //events
            while (SDL_PollEvent(&e) != 0)
            {
                if (e.type == SDL_QUIT)
                {
                    quit = true;
                }
                else if (e.type == SDL_MOUSEMOTION)
                {
                    SDL_GetMouseState(&x, &y);
                    //std::cout << "Mouse position: (" << x << ", " << y << ")" << std::endl;
                    
                }
                else if (e.type == SDL_MOUSEBUTTONDOWN)
                {
                    std::cout << "CLICK\n";
                    x_div = x / 128.0f;
                    y_div = y / 128.0f;

                    goal_state << x_div, y_div, 0, 0, 0, 0;

                    quadrotor.SetGoal(goal_state);
                    K = LQR(quadrotor, dt);
                } 
                else if (e.type == SDL_KEYDOWN) 
                {
                    if(e.key.keysym.sym==SDLK_TAB){
                        std::cout << "TAB\n";
                        auto controlState = quadrotor.GetControlState();
                        auto state = quadrotor.GetState();
                        for (const auto& element : controlState) {
                            std::cout << element << ' ';
                        }
                        std::cout << '\n';
                        for (const auto& element : state) {
                            std::cout << element << ' ';
                        }
                        std::cout << '\n';
                    }
                    else if(e.key.keysym.sym==SDLK_p){
                        std::cout << "P\n";
                        matplot::subplot(3, 3, 0);
                        matplot::ylim({0,SCREEN_WIDTH/128.});
                        matplot::plot(time_history, x_history);
                        matplot::title("X over time");
                        matplot::xlabel("Time [ms]");
                        matplot::ylabel("X [m]");
                        matplot::grid(true);
                        
                        matplot::subplot(3, 3, 3);
                        matplot::ylim({0,SCREEN_HEIGHT/128.});
                        matplot::plot(time_history, y_history);
                        matplot::title("Y over time");
                        matplot::xlabel("Time [ms]");
                        matplot::ylabel("Y [m]");
                        matplot::grid(true);
                        
                        matplot::subplot(3, 3, {6, 7, 8});
                        matplot::ylim({-1,1});
                        matplot::plot(time_history, theta_history);
                        matplot::title("Theta over time");
                        matplot::xlabel("Time [ms]");
                        matplot::ylabel("Theta [rad]");
                        matplot::grid(true);
                        
                        matplot::subplot(3, 3, {1, 2, 4, 5});
                        matplot::ylim({0,SCREEN_HEIGHT/128.});
                        matplot::xlim({0,SCREEN_WIDTH/128.});
                        matplot::plot(x_history, y_history);
                        matplot::title("X-Y trajectory");
                        matplot::xlabel("X [m]");
                        matplot::ylabel("Y [m]");
                        matplot::grid(true);

                        matplot::show();
                    }
                }
            }

            SDL_Delay((int) dt * 1000);

            SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
            SDL_RenderClear(gRenderer.get());

            /* Quadrotor rendering step */
            quadrotor_visualizer.render(gRenderer, animationCounter);
            animationCounter++;
            animationCounter %= (int) (1. / dt);

            SDL_RenderPresent(gRenderer.get());

            /* Simulate quadrotor forward in time */
            control(quadrotor, K);
            quadrotor.Update(dt);

            if(vectorsCounter == 90){
                x_history.push_back(quadrotor.GetState()[0]);
                y_history.push_back(SCREEN_HEIGHT/128.-quadrotor.GetState()[1]);
                theta_history.push_back(quadrotor.GetState()[2]);
                time_history.push_back(time_history.back() + dt * 1000);
                vectorsCounter = 0;
            }
            vectorsCounter++;
        }
    }
    x_history.clear();
    y_history.clear();
    theta_history.clear();
    time_history.clear();
    SDL_Quit();
    return 0;
}

int init(std::shared_ptr<SDL_Window>& gWindow, std::shared_ptr<SDL_Renderer>& gRenderer, const int SCREEN_WIDTH, const int SCREEN_HEIGHT)
{
    if (SDL_Init(SDL_INIT_VIDEO) >= 0)
    {
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
        gWindow = std::shared_ptr<SDL_Window>(SDL_CreateWindow("Planar Quadrotor", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN), SDL_DestroyWindow);
        gRenderer = std::shared_ptr<SDL_Renderer>(SDL_CreateRenderer(gWindow.get(), -1, SDL_RENDERER_ACCELERATED), SDL_DestroyRenderer);
        SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
    }
    else
    {
        std::cout << "SDL_ERROR: " << SDL_GetError() << std::endl;
        return -1;
    }
    return 0;
}
