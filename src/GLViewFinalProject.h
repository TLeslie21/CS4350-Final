#pragma once

#include "GLView.h"
#include "NetMessengerClient.h"
#include "PxPhysicsAPI.h"
#include "vehicle/PxVehicleSDK.h"
#include "extensions/PxDefaultErrorCallback.h"
#include "extensions/PxDefaultAllocator.h"
#include "irrKlang.h"
#include <ctime>
#include <chrono>
#include <thread>


namespace Aftr
{
    class Camera;

    /**
       \class GLViewFinalProject
       \author Scott Nykl
       \brief A child of an abstract GLView. This class is the top-most manager of the module.

       Read \see GLView for important constructor and init information.

       \see GLView

        \{
    */

    class GLViewFinalProject : public GLView
    {
    public:
        static GLViewFinalProject* New(const std::vector< std::string >& outArgs);
        virtual ~GLViewFinalProject();
        virtual void updateWorld(); ///< Called once per frame
        virtual void loadMap(); ///< Called once at startup to build this module's scene
        virtual void onResizeWindow(GLsizei width, GLsizei height);
        virtual void onMouseDown(const SDL_MouseButtonEvent& e);
        virtual void onMouseUp(const SDL_MouseButtonEvent& e);
        virtual void onMouseMove(const SDL_MouseMotionEvent& e);
        virtual void createFinalProjectWayPoints();
        virtual void addYRoad(float length, float x, float y);
        virtual void addXRoad(float length, float x, float y);
        virtual void placeCoins(float x, float y, float z);
        virtual void removeAllCoins();
        virtual void placeBarrier(float rotation, float x, float y);
        virtual bool isCollisionDetected(WO* car, WO* coin);
        virtual void startGame();
        virtual int endGame();
        virtual char* getScore1();
        virtual char* getScore2();
        virtual void onKeyDown(const SDL_KeyboardEvent& key);
        virtual void onKeyUp(const SDL_KeyboardEvent& key);

        bool w = false;
        bool a = false;
        bool s = false;
        bool d = false;
        bool space = false;

        NetMessengerClient* client = nullptr;
        Vector position;
        bool player1Turn = false;
        bool player2Turn = false;
        bool oneWent = false;
        bool twoWent = false;
        int score1 = 0;
        int score2 = 0;
        const float collectionRadius = 1.0f;
        bool gameOver;
        std::vector<WO*> activeCoins;
        int seconds = 30;
        bool timerActive;
        float remainingTime;

        // PhysX //
        WO* coin;
        physx::PxShape* coinShape = nullptr;
        physx::PxRigidDynamic* coinActor;
        
        WO* soccerBall;
        physx::PxShape* soccerBallShape = nullptr;
        physx::PxRigidDynamic* soccerBallActor;
        
        WO* concreteBarrier;
        physx::PxShape* concreteBShape = nullptr;
        physx::PxRigidDynamic* concreteActor;
        
        physx::PxDefaultAllocator defaultAllocator;
        physx::PxDefaultErrorCallback e;
        physx::PxFoundation* f;
        physx::PxPhysics* p;
        physx::PxScene* scene;
        physx::PxMaterial* gMaterial;

    protected:
        GLViewFinalProject(const std::vector< std::string >& args);
        virtual void onCreate();
        irrklang::ISoundEngine* soundEngine;
        irrklang::ISound* sound;
    };

} //namespace Aftr