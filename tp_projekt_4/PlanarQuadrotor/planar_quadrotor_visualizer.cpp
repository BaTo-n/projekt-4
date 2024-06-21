#include "planar_quadrotor_visualizer.h"

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor *quadrotor_ptr): quadrotor_ptr(quadrotor_ptr) {}

/**
 * TODO: Improve visualizetion
 * 1. Transform coordinates from quadrotor frame to image frame so the circle is in the middle of the screen
 * 2. Use more shapes to represent quadrotor (e.x. try replicate http://underactuated.mit.edu/acrobot.html#section3 or do something prettier)
 * 3. Animate proppelers (extra points)
 */
void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer> &gRenderer, int &animation_state) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    float q_x, q_y, q_theta;


    /* x, y, theta coordinates */
    q_x = state[0];
    q_y = state[1];
    q_theta = state[2];

    int OsX = (int)(q_x * 128.0f);
    int OsY = (int)(q_y * 128.0f);

    SDL_Rect body;
    body.x = OsX - 30 ;
    body.y = OsY - 10;
    body.w = 60;
    body.h = 20;
    SDL_Rect kij1;
    kij1.x = body.x - 5;
    kij1.y = body.y - 20;
    kij1.w = 5;
    kij1.h = 40;
    SDL_Rect kij2;
    kij2.x = body.x + body.w;
    kij2.y = body.y - 20;
    kij2.w = 5;
    kij2.h = 40;

    if(animation_state % 50 / 20){
        filledEllipseColor(gRenderer.get(), kij1.x - 13, kij1.y + 3, 15, 3, 0xFF4e77FF); //0xFF4e77FF
        filledEllipseColor(gRenderer.get(), kij1.x + 5 + 13, kij1.y + 3, 15, 3, 0xFF4e77FF);
        filledEllipseColor(gRenderer.get(), kij2.x - 13, kij2.y + 3, 15, 3, 0xFF4e77FF);
        filledEllipseColor(gRenderer.get(), kij2.x + 5 + 13, kij2.y + 3, 15, 3, 0xFF4e77FF);
    }
    
    SDL_SetRenderDrawColor(gRenderer.get(), 0x16, 0x8a, 0xad, 0xFF); //16 8a ad
    SDL_RenderFillRect(gRenderer.get(), &body);
    SDL_SetRenderDrawColor(gRenderer.get(), 0x34, 0xA0, 0xa4, 0xFF); //34A0A4
    SDL_RenderFillRect(gRenderer.get(), &kij1); 
    SDL_RenderFillRect(gRenderer.get(), &kij2);
}
