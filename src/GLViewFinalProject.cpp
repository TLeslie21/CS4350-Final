#include "GLViewFinalProject.h"

#include "WorldList.h" //This is where we place all of our WOs
#include "ManagerOpenGLState.h" //We can change OpenGL State attributes with this
#include "Axes.h" //We can set Axes to on/off with this
#include "PhysicsEngineODE.h"

//Different WO used by this module
#include "WO.h"
#include "WOStatic.h"
#include "WOStaticPlane.h"
#include "WOStaticTrimesh.h"
#include "WOTrimesh.h"
#include "WOHumanCyborg.h"
#include "WOHumanCal3DPaladin.h"
#include "WOWayPointSpherical.h"
#include "WOLight.h"
#include "WOSkyBox.h"
#include "WOCar1970sBeater.h"
#include "Camera.h"
#include "CameraStandard.h"
#include "CameraChaseActorSmooth.h"
#include "CameraChaseActorAbsNormal.h"
#include "CameraChaseActorRelNormal.h"
#include "Model.h"
#include "ModelDataShared.h"
#include "ModelMesh.h"
#include "ModelMeshDataShared.h"
#include "ModelMeshSkin.h"
#include "WONVStaticPlane.h"
#include "WONVPhysX.h"
#include "WONVDynSphere.h"
#include "WOImGui.h" //GUI Demos also need to #include "AftrImGuiIncludes.h"
#include "AftrImGuiIncludes.h"
#include "AftrGLRendererBase.h"
#include "VectorFwd.h"
#include "NetMsg.h"
#include "NetMessengerClient.h"
#include "NetMsgCreateWO.h"
#include "PxPhysicsAPI.h"
#include "vehicle/PxVehicleSDK.h"
#include "extensions/PxDefaultErrorCallback.h"
#include "extensions/PxDefaultAllocator.h"
#include "irrKlang.h"
#include <iostream>
#include <ctime>
#include <chrono>
#include <thread>

using namespace irrklang;
using namespace Aftr;
using namespace std::chrono;
using namespace std::this_thread;

GLViewFinalProject* GLViewFinalProject::New(const std::vector< std::string >& args)
{
    GLViewFinalProject* glv = new GLViewFinalProject(args);
    glv->init(Aftr::GRAVITY, Vector(0, 0, -1.0f), "aftr.conf", PHYSICS_ENGINE_TYPE::petODE);
    glv->onCreate();
    return glv;
}

GLViewFinalProject::GLViewFinalProject(const std::vector< std::string >& args) : GLView(args)
{
    //Initialize any member variables that need to be used inside of LoadMap() here.
    //Note: At this point, the Managers are not yet initialized. The Engine initialization
    //occurs immediately after this method returns (see GLViewFinalProject::New() for
    //reference). Then the engine invoke's GLView::loadMap() for this module.
    //After loadMap() returns, GLView::onCreate is finally invoked.

    //The order of execution of a module startup:
    //GLView::New() is invoked:
    //    calls GLView::init()
    //       calls GLView::loadMap() (as well as initializing the engine's Managers)
    //    calls GLView::onCreate()

    //GLViewFinalProject::onCreate() is invoked after this module's LoadMap() is completed.
    soundEngine = createIrrKlangDevice();
    sound = nullptr;
}

void GLViewFinalProject::onCreate()
{
    //GLViewFinalProject::onCreate() is invoked after this module's LoadMap() is completed.
    //At this point, all the managers are initialized. That is, the engine is fully initialized.

    if (this->pe != NULL)
    {
        //optionally, change gravity direction and magnitude here
        //The user could load these values from the module's aftr.conf
        this->pe->setGravityNormalizedVector(Vector(0, 0, -1.0f));
        this->pe->setGravityScalar(Aftr::GRAVITY);
    }
    this->setActorChaseType(STANDARDEZNAV); //Default is STANDARDEZNAV mode
    //this->setNumPhysicsStepsPerRender( 0 ); //pause physics engine on start up; will remain paused till set to 1
}

GLViewFinalProject::~GLViewFinalProject()
{
    //Implicitly calls GLView::~GLView()
    if (soundEngine)
    {
        soundEngine->drop();
        soundEngine = nullptr;
    }
}

void GLViewFinalProject::updateWorld()
{
    GLView::updateWorld(); //Just call the parent's update world first.
    //If you want to add additional functionality, do it after
    //this call.

    Vector pos = this->cam->getPosition();
    Vector look = this->cam->getLookDirection();
    Vector up = this->cam->getNormalDirection();

    // (Flip Z axes)
    soundEngine->setListenerPosition(
        vec3df(pos.x, pos.y, -pos.z),
        vec3df(look.x, look.y, -look.z),
        vec3df(0, 0, 0),
        vec3df(up.x, up.y, -up.z)
    );

    scene->simulate(0.1);
    physx::PxU32 errorState = 0;
    scene->fetchResults(true);
    // Gravity gives objects some weight to keep them from floating forever
    scene->setGravity(physx::PxVec3(0, 0, -6.5));
    physx::PxVec3 totalForce(0, 0, 0);

    soccerBall->setPosition(Vector(soccerBallActor->getGlobalPose().p.x, soccerBallActor->getGlobalPose().p.y, soccerBallActor->getGlobalPose().p.z));
    this->cam->setPosition(Vector(soccerBallActor->getGlobalPose().p.x, soccerBallActor->getGlobalPose().p.y, soccerBallActor->getGlobalPose().p.z + 50));
    this->cam->setCameraLookAtPoint(Vector(soccerBallActor->getGlobalPose().p.x, soccerBallActor->getGlobalPose().p.y, soccerBallActor->getGlobalPose().p.z));

    // Check for collisions
    for (auto it = activeCoins.begin(); it != activeCoins.end(); ) {
        WO* coin = *it;
        if (coin != nullptr && isCollisionDetected(soccerBall, coin)) {
            // Hide coin
            coin->setPosition(Vector(10, 0, -5));
            it = activeCoins.erase(it);
        }
        else {
            ++it;
        }
    }

    //  Move up
    if (w == true) {
        totalForce += physx::PxVec3(0, -4, 0);
    }
    // Move down
    if (s == true) {
        totalForce += physx::PxVec3(0, 4, 0);
    }

    // Move left
    if (a == true) {
        totalForce += physx::PxVec3(4, 0, 0);
    }
    // Move right
    if (d == true) {
        totalForce += physx::PxVec3(-4, 0, 0);
    }
    if (totalForce.magnitude() > 0) {
        totalForce = totalForce.getNormalized() * 4;
    }

    soccerBallActor->addForce(totalForce);

    if (timerActive) {
        remainingTime -= 1.0f / 60.0f;
        if (remainingTime <= 0) {
            remainingTime = 0;
            timerActive = false;
        }
    }
}

void GLViewFinalProject::onResizeWindow(GLsizei width, GLsizei height)
{
    GLView::onResizeWindow(width, height); //call parent's resize method.
}

void GLViewFinalProject::onMouseDown(const SDL_MouseButtonEvent& e)
{
    GLView::onMouseDown(e);
}

void GLViewFinalProject::onMouseUp(const SDL_MouseButtonEvent& e)
{
    GLView::onMouseUp(e);
}

void GLViewFinalProject::onMouseMove(const SDL_MouseMotionEvent& e)
{
    GLView::onMouseMove(e);
}

void GLViewFinalProject::addYRoad(float length, float x, float y) {
    std::string road(ManagerEnvironmentConfiguration::getLMM() + "/models/untitled.obj");
    WO* straightRoad = WO::New(road, Vector(length, 2.5, 1), MESH_SHADING_TYPE::mstSMOOTH);
    straightRoad->setPosition(x, y, 0.1);
    straightRoad->getModel()->rotateAboutRelZ(-90 * (PI / 180));
    straightRoad->upon_async_model_loaded([straightRoad]()
        {
            std::string texPath(ManagerEnvironmentConfiguration::getLMM() + "/images/textures/render5.png");
            ModelMeshSkin roadSkin(ManagerTex::loadTexAsync(texPath).value());
            roadSkin.setMeshShadingType(MESH_SHADING_TYPE::mstSMOOTH);
            straightRoad->getModel()->getModelDataShared()->getModelMeshes().at(0)->addSkin(std::move(roadSkin));
            straightRoad->getModel()->getModelDataShared()->getModelMeshes().at(0)->useNextSkin();
        });
    straightRoad->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    straightRoad->setLabel("Road");
    this->worldLst->push_back(straightRoad);
}

void GLViewFinalProject::addXRoad(float length, float x, float y) {
    std::string road(ManagerEnvironmentConfiguration::getLMM() + "/models/untitled.obj");
    WO* straightRoad = WO::New(road, Vector(length, 2.5, 1), MESH_SHADING_TYPE::mstSMOOTH);
    straightRoad->setPosition(x, y, 0.1);
    straightRoad->upon_async_model_loaded([straightRoad]()
        {
            std::string texPath(ManagerEnvironmentConfiguration::getLMM() + "/images/textures/render5.png");
            ModelMeshSkin roadSkin(ManagerTex::loadTexAsync(texPath).value());
            roadSkin.setMeshShadingType(MESH_SHADING_TYPE::mstSMOOTH);
            straightRoad->getModel()->getModelDataShared()->getModelMeshes().at(0)->addSkin(std::move(roadSkin));
            straightRoad->getModel()->getModelDataShared()->getModelMeshes().at(0)->useNextSkin();
        });
    straightRoad->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    straightRoad->setLabel("Road");
    this->worldLst->push_back(straightRoad);
}

void GLViewFinalProject::placeCoins(float x, float y, float z) {
    std::string coinString(ManagerEnvironmentConfiguration::getLMM() + "/models/Coin_Star.fbx");
    coin = WO::New(coinString, Vector(.03, .03, .03), MESH_SHADING_TYPE::mstSMOOTH);
    coin->setPosition(Vector(x, y, z));
    coin->setLabel("Coin");
    coin->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    this->worldLst->push_back(coin);
    activeCoins.push_back(coin);
}

void GLViewFinalProject::removeAllCoins() {
    for (auto it = activeCoins.begin(); it != activeCoins.end(); ) {
        WO* coin = *it;
        if (coin != nullptr) {
            // Hide coin
            coin->setPosition(Vector(10, 0, -5));
            it = activeCoins.erase(it);
        }
        else {
            ++it;
        }
    }
}

void GLViewFinalProject::placeBarrier(float rotation, float x, float y) {
    std::string concreteBarrierString(ManagerEnvironmentConfiguration::getLMM() + "/models/63-concrete-barrier/CONCRETE BARRIER.fbx");
    WO* concreteBarrier = WO::New(concreteBarrierString, Vector(.001, .001, .001), MESH_SHADING_TYPE::mstSMOOTH);
    concreteBarrier->setPosition(x, y, 1.0);
    concreteBarrier->getModel()->rotateAboutRelZ(rotation * (PI / 180));
    concreteBarrier->upon_async_model_loaded([concreteBarrier]()
        {
            std::string concBarrierPath(ManagerEnvironmentConfiguration::getLMM() + "/images/uploads_files_4096737_textures/textures/Concrete020_2K-PNG/Concrete020_2K_Color.png");
            ModelMeshSkin concreteTex(ManagerTex::loadTexAsync(concBarrierPath).value());
            concreteTex.setMeshShadingType(MESH_SHADING_TYPE::mstSMOOTH);
            concreteBarrier->getModel()->getModelDataShared()->getModelMeshes().at(0)->addSkin(std::move(concreteTex));
            concreteBarrier->getModel()->getModelDataShared()->getModelMeshes().at(0)->useNextSkin();
        });
    concreteBarrier->setLabel("Concrete Barrier");
    concreteBarrier->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
    this->worldLst->push_back(concreteBarrier);
    gMaterial = p->createMaterial(0.5f, 0.5f, 0.6f);
    concreteBShape = p->createShape(physx::PxBoxGeometry(1.44, 1.60, 1.0), *gMaterial);
    physx::PxTransform t({ x, y, 1.0 });
    concreteActor = p->createRigidDynamic(t);
    concreteActor->attachShape(*concreteBShape);
    concreteActor->setMass(30);
    scene->addActor(*concreteActor);
}

bool GLViewFinalProject::isCollisionDetected(WO* soccerBall, WO* coin) {
    
    Vector soccerBallPos = soccerBall->getPosition();
    Vector coinPos = coin->getPosition();

    float soccerBallRadius = 1.5f;
    float coinRadius = 1.5f;

    float distance = (soccerBallPos - coinPos).magnitude();
    if (distance < (soccerBallRadius + coinRadius)) {
        if (player1Turn) {
            score1 += 1;
            if (coin != nullptr) {
                coin->setPosition((10, 0, -5));
            }
        }
        else if (player2Turn) {
            score2 += 1;
            if (coin != nullptr) {
                coin->setPosition((10, 0, -5));
            }
        }
    }
    return distance < (soccerBallRadius + coinRadius);
}

void GLViewFinalProject::startGame() {
    score1 = 0; // Reset score
    score1 = 0; // Reset score
    gameOver = false;
}

int GLViewFinalProject::endGame() {
    gameOver = true;
    if (score1 > score2) {
        return 1;  // Player 1 wins
    }
    if (score1 == score2) {
        return 0;  // Tie
    }
    return 2;  // Player 2 wins
}

char* GLViewFinalProject::getScore1() {
    char scoreStr[5];
    sprintf(scoreStr, "%d", score1);
    return scoreStr;
}

char* GLViewFinalProject::getScore2() {
    char scoreStr[5];
    sprintf(scoreStr, "%d", score2);
    return scoreStr;
}

void GLViewFinalProject::onKeyDown(const SDL_KeyboardEvent& key)
{
    GLView::onKeyDown(key);
    if (key.keysym.sym == SDLK_0)
        this->setNumPhysicsStepsPerRender(1);
    if (key.keysym.sym == SDLK_1)
    {
        NetMsgCreateWO msg;
        position = soccerBall->getPosition();
        msg.xPos = position.x;
        msg.yPos = position.y;
        msg.zPos = position.z;
        client->sendNetMsgSynchronousTCP(msg);
    }
    
    if (key.keysym.sym == SDLK_w) { w = true; }
    if (key.keysym.sym == SDLK_a) { a = true; }
    if (key.keysym.sym == SDLK_s) { s = true; }
    if (key.keysym.sym == SDLK_d) { d = true; }
    if (key.keysym.sym == SDLK_SPACE) { space = true; }
}

void GLViewFinalProject::onKeyUp(const SDL_KeyboardEvent& key)
{
    GLView::onKeyUp(key);
    if (key.keysym.sym == SDLK_w)
    {
        w = false;
    }
    else if (key.keysym.sym == SDLK_a)
    {
        a = false;
    }
    else if (key.keysym.sym == SDLK_d)
    {
        d = false;
    }
    else if (key.keysym.sym == SDLK_s)
    {
        s = false;
    }
    else if (key.keysym.sym == SDLK_SPACE) {
        space = false;
    }
}

void Aftr::GLViewFinalProject::loadMap()
{
    this->worldLst = new WorldList(); //WorldList is a 'smart' vector that is used to store WO*'s
    this->actorLst = new WorldList();
    this->netLst = new WorldList();

    ManagerOpenGLState::GL_CLIPPING_PLANE = 1000.0;
    ManagerOpenGLState::GL_NEAR_PLANE = 0.1f;
    ManagerOpenGLState::enableFrustumCulling = false;
    Axes::isVisible = false;
    this->glRenderer->isUsingShadowMapping(false); //set to TRUE to enable shadow mapping, must be using GL 3.2+

    this->cam->setPosition(25, 20, 15);
    this->cam->setCameraLookAtPoint(Vector(25, -2.5, 2.5));

    client = NetMessengerClient::New("127.0.0.1", "12680");
    sound = soundEngine->play2D("C:/repos/aburn/usr/modules/FinalProject/mm/irrKlang/media/chirp.wav", true);
    soundEngine->setSoundVolume(.01);

    f = PxCreateFoundation(PX_PHYSICS_VERSION, defaultAllocator, e);
    p = PxCreatePhysics(PX_PHYSICS_VERSION, *f, physx::PxTolerancesScale(), true, NULL);
    physx::PxSceneDesc sc(p->getTolerancesScale());
    sc.filterShader = physx::PxDefaultSimulationFilterShader;
    sc.cpuDispatcher = physx::PxDefaultCpuDispatcherCreate(2);
    scene = p->createScene(sc);
    scene->setFlag(physx::PxSceneFlag::eENABLE_ACTIVE_ACTORS, true);

    std::string shinyRedPlasticCube(ManagerEnvironmentConfiguration::getSMM() + "/models/cube4x4x4redShinyPlastic_pp.wrl");
    std::string book(ManagerEnvironmentConfiguration::getSMM() + "/models/book.wrl");
    std::string wheeledCar(ManagerEnvironmentConfiguration::getSMM() + "/models/rcx_treads.wrl");
    std::string grass(ManagerEnvironmentConfiguration::getSMM() + "/models/grassFloor400x400_pp.wrl");
    std::string human(ManagerEnvironmentConfiguration::getSMM() + "/models/human_chest.wrl");
    std::string carModel(ManagerEnvironmentConfiguration::getLMM() + "/models/Car.3ds");
    std::string road(ManagerEnvironmentConfiguration::getLMM() + "/models/untitled.obj");
    std::string coinString(ManagerEnvironmentConfiguration::getLMM() + "/models/Coin_Star.fbx");
    std::string concreteBarrierString(ManagerEnvironmentConfiguration::getLMM() + "/models/63-concrete-barrier/CONCRETE BARRIER.fbx");
    std::string footballString(ManagerEnvironmentConfiguration::getLMM() + "/models/Football_ball.fbx");

    ///SkyBox Textures readily available
    std::vector< std::string > skyBoxImageNames; //vector to store texture paths
    skyBoxImageNames.push_back(ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_afternoon+6.jpg");

    {
        //Create a light
        float ga = 0.1f; //Global Ambient Light level for this module
        ManagerLight::setGlobalAmbientLight(aftrColor4f(ga, ga, ga, 1.0f));
        WOLight* light = WOLight::New();
        light->isDirectionalLight(true);
        light->setPosition(Vector(0, 0, 100));
        //Set the light's display matrix such that it casts light in a direction parallel to the -z axis (ie, downwards as though it was "high noon")
        //for shadow mapping to work, this->glRenderer->isUsingShadowMapping( true ), must be invoked.
        light->getModel()->setDisplayMatrix(Mat4::rotateIdentityMat({ 0, 1, 0 }, 90.0f * Aftr::DEGtoRAD));
        light->setLabel("Light");
        worldLst->push_back(light);
    }

    {
        //Create the SkyBox
        WO* wo = WOSkyBox::New(skyBoxImageNames.at(0), this->getCameraPtrPtr());
        wo->setPosition(Vector(0, 0, 0));
        wo->setLabel("Sky Box");
        wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
        worldLst->push_back(wo);
    }

    {
        //Create the infinite grass plane (the floor)
        WO* wo = WO::New(grass, Vector(1, 1, 1), MESH_SHADING_TYPE::mstFLAT);
        wo->setPosition(Vector(0, 0, 0));
        wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
        wo->upon_async_model_loaded([wo]()
            {
                ModelMeshSkin& grassSkin = wo->getModel()->getModelDataShared()->getModelMeshes().at(0)->getSkins().at(0);
                grassSkin.getMultiTextureSet().at(0).setTexRepeats(5.0f);
                grassSkin.setAmbient(aftrColor4f(0.4f, 0.4f, 0.4f, 1.0f)); //Color of object when it is not in any light
                grassSkin.setDiffuse(aftrColor4f(1.0f, 1.0f, 1.0f, 1.0f)); //Diffuse color components (ie, matte shading color of this object)
                grassSkin.setSpecular(aftrColor4f(0.4f, 0.4f, 0.4f, 1.0f)); //Specular color component (ie, how "shiney" it is)
                grassSkin.setSpecularCoefficient(10); // How "sharp" are the specular highlights (bigger is sharper, 1000 is very sharp, 10 is very dull)
            });
        wo->setLabel("Grass");
        worldLst->push_back(wo);

        gMaterial = p->createMaterial(0.5f, 0.5f, 0.6f);
        physx::PxRigidStatic* groundPlane = PxCreatePlane(*p, physx::PxPlane(0, 0, 1, 0), *gMaterial);//good for the grass
        scene->addActor(*groundPlane);
    }

    // Place roads
    // Length, X, Y
    addYRoad(5, 25, -75);
    addXRoad(5, 25, -159.5);
    addYRoad(5, 98.8, -75);
    addYRoad(1.25, 78.8, 35);
    addYRoad(1.75, 7.2, 32.7);
    addXRoad(4.9, 90.25, 55);
    addYRoad(5, -48.9, -75);
    addXRoad(5, 25, 9.65);
    addXRoad(5, -59.5, -81.9);
    addXRoad(5, 99.5, -39.9);
    addYRoad(3.5, 56.6, -100);
    addYRoad(3, 173.35, 13);
    addYRoad(5.5, -133.5, -35);
    addXRoad(3, -91.3, -127);
    addXRoad(3, -91.3, -27);
    addXRoad(4.45, -68.5, 55);
    addYRoad(3.98, -88.0, -13.45);

    placeBarrier(-90, 27.5, -20);
    placeBarrier(-90, 23.5, -80);
    placeBarrier(90, -50, -26);
    placeBarrier(0, -78.5, -28);
    placeBarrier(90, 173.5, -18);
    placeBarrier(0, -80.5, -80);
    placeBarrier(90, 98.5, -58);

 
    // Soccer ball
    {
        soccerBall = WO::New(footballString, Vector( .1, .1, .1 ), MESH_SHADING_TYPE::mstSMOOTH);
        soccerBall->setPosition(25, 5, 2.5);
        soccerBall->setLabel("soccerBall");
        soccerBall->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
        this->worldLst->push_back(soccerBall);
        gMaterial = p->createMaterial(0.5f, 0.55f, 0.3f);
        soccerBallShape = p->createShape(physx::PxBoxGeometry(1.1, 1.1, 1.1), *gMaterial);
        physx::PxTransform t({ 25, -2.5, 2.5 });
        soccerBallActor = p->createRigidDynamic(t);
        soccerBallActor->attachShape(*soccerBallShape);
        soccerBallActor->setMass(1);
        scene->addActor(*soccerBallActor);
    }
    
    WOImGui* gui = WOImGui::New(nullptr);
    gui->setLabel("My Gui");
    gui->subscribe_drawImGuiWidget(
        [this, gui, footballString]() //this is a lambda, the capture clause is in [], the input argument list is in (), and the body is in {}
        {
            // Declare variables
            Vector position = soccerBall->getPosition();

            if (ImGui::Begin("Final Project")) {
                ImGui::StyleColorsDark();
                ImGui::Text("Coins collected:");
                ImGui::Text("Player 1: ");
                ImGui::Text(getScore1());
                ImGui::Text("Player 2: ");
                ImGui::Text(getScore2());
                ImGui::Separator();
                ImGui::Text("Time Remaining: %.1f", remainingTime);

                if (remainingTime == 0 && (oneWent == true && twoWent == true)) {
                    if (endGame() == 1) {
                        ImGui::Text("Player 1 wins!");
                    }
                    if (endGame() == 2) {
                        ImGui::Text("Player 2 wins!");
                    }
                    if (endGame() == 0) {
                        ImGui::Text("It's a tie!");
                    }
                }

                // Player 1
                if (ImGui::Button("Start Player 1's Turn")) {
                    player1Turn = true;
                    player2Turn = false;
                    oneWent = true;

                    if (soccerBall != nullptr && soccerBallActor != nullptr) {
                        this->worldLst->eraseViaWOptr(soccerBall);
                        scene->removeActor(*soccerBallActor);
                        delete soccerBall;
                        soccerBall = nullptr;
                        soccerBallActor->release();
                        soccerBallActor = nullptr;
                    }
                    w = false;
                    a = false;
                    s = false;
                    d = false;
                    space = false;
                    soccerBall = WO::New(footballString, Vector(.1, .1, .1), MESH_SHADING_TYPE::mstSMOOTH);
                    soccerBall->setPosition(25, 5, 2.5);
                    soccerBall->setLabel("soccerBall");
                    soccerBall->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
                    this->worldLst->push_back(soccerBall);
                    gMaterial = p->createMaterial(0.5f, 0.55f, 0.3f);
                    soccerBallShape = p->createShape(physx::PxBoxGeometry(1.1, 1.1, 1.1), *gMaterial);
                    physx::PxTransform t({ 25, -2.5, 2.5 });
                    soccerBallActor = p->createRigidDynamic(t);
                    soccerBallActor->attachShape(*soccerBallShape);
                    scene->addActor(*soccerBallActor);

                    std::vector<Vector> coinPositions = {
                        Vector(25.0f, -50.0f, 1.5f),
                        Vector(97.8f, -74.7f, 1.5f),
                        Vector(-49.0f, -70.3f, 1.5f),
                        Vector(23.0f, -25.0f, 1.5f),
                        Vector(56.1f, -90.0f, 1.5f),
                        Vector(27.0f, -120.0f, 1.5f),
                        Vector(55.0f, 10.0f, 1.5f),
                        Vector(100.5f, -5.0f, 1.5f),
                        Vector(-86.0f, -45.0f, 1.5f),
                        Vector(-90.0f, 25.0f, 1.5f),
                        Vector(-132.5f, -25.0f, 1.5f),
                        Vector(-136.0f, 35.0f, 1.5f),
                        Vector(-134.5f, -85.0f, 1.5f),
                        Vector(-135.5f, -60.0f, 1.5f),
                        Vector(-30.5f, 55.0f, 1.5f),
                        Vector(25.5f, 58.0f, 1.5f),
                        Vector(120.5f, 52.5f, 1.5f),
                        Vector(45.5f, 57.5f, 1.5f),
                        Vector(-90.5f, 59.0f, 1.5f),
                        Vector(170.5f, 40.0f, 1.5f),
                        Vector(175.5f, -10.0f, 1.5f),
                        Vector(75.5f, 25.0f, 1.5f),
                        Vector(80.5f, 45.0f, 1.5f),
                        Vector(140.5f, -41.0f, 1.5f),
                        Vector(45.5f, -38.0f, 1.5f),
                        Vector(85.5f, -40.0f, 1.5f),
                        Vector(-50.5f, -130.0f, 1.5f),
                        Vector(-46.5f, -20.0f, 1.5f),
                        Vector(-20.0f, -160.0f, 1.5f),
                        Vector(7.5f, -163.2f, 1.5f),
                        Vector(35.5f, -159.0f, 1.5f),
                        Vector(100.5f, -160.0f, 1.5f),
                        Vector(-5.0f, 7.0f, 1.5f),
                        Vector(7.0f, 40.0f, 1.5f),
                        Vector(-7.0f, -85.0f, 1.5f),
                        Vector(-107.0f, -81.5f, 1.5f),
                        Vector(-67.0f, -84.0f, 1.5f),
                        Vector(-67.0f, -29.0f, 1.5f),
                        Vector(-107.0f, -25.0f, 1.5f),
                        Vector(-100.0f, -124.0f, 1.5f),
                    };

                    // Place coins
                    for (const auto& pos : coinPositions) {
                        placeCoins(pos.x, pos.y, pos.z);
                    }

                    // Start the timer
                    remainingTime = 10.0f;
                    timerActive = true;
                    
                }
                if (ImGui::Button("Start Player 2's Turn")) {
                    player1Turn = false;
                    player2Turn = true;
                    twoWent = true;

                    if (soccerBall != nullptr && soccerBallActor != nullptr) {
                        this->worldLst->eraseViaWOptr(soccerBall);
                        scene->removeActor(*soccerBallActor);
                        delete soccerBall;
                        soccerBall = nullptr;
                        soccerBallActor->release();
                        soccerBallActor = nullptr;
                    }
                    w = false;
                    a = false;
                    s = false;
                    d = false;
                    space = false;
                    soccerBall = WO::New(footballString, Vector(.1, .1, .1), MESH_SHADING_TYPE::mstSMOOTH);
                    soccerBall->setPosition(25, 5, 2.5);
                    soccerBall->setLabel("soccerBall");
                    soccerBall->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
                    this->worldLst->push_back(soccerBall);
                    gMaterial = p->createMaterial(0.5f, 0.55f, 0.3f);
                    soccerBallShape = p->createShape(physx::PxBoxGeometry(1.1, 1.1, 1.1), *gMaterial);
                    physx::PxTransform t({ 25, -2.5, 2.5 });
                    soccerBallActor = p->createRigidDynamic(t);
                    soccerBallActor->attachShape(*soccerBallShape);
                    scene->addActor(*soccerBallActor);

                    removeAllCoins();
                    std::vector<Vector> coinPositions = {
                        Vector(25.0f, -50.0f, 1.5f),
                        Vector(97.8f, -74.7f, 1.5f),
                        Vector(-49.0f, -70.3f, 1.5f),
                        Vector(23.0f, -25.0f, 1.5f),
                        Vector(56.1f, -90.0f, 1.5f),
                        Vector(27.0f, -120.0f, 1.5f),
                        Vector(55.0f, 10.0f, 1.5f),
                        Vector(100.5f, -5.0f, 1.5f),
                        Vector(-86.0f, -45.0f, 1.5f),
                        Vector(-90.0f, 25.0f, 1.5f),
                        Vector(-132.5f, -25.0f, 1.5f),
                        Vector(-136.0f, 35.0f, 1.5f),
                        Vector(-134.5f, -85.0f, 1.5f),
                        Vector(-135.5f, -60.0f, 1.5f),
                        Vector(-30.5f, 55.0f, 1.5f),
                        Vector(25.5f, 58.0f, 1.5f),
                        Vector(120.5f, 52.5f, 1.5f),
                        Vector(45.5f, 57.5f, 1.5f),
                        Vector(-90.5f, 59.0f, 1.5f),
                        Vector(170.5f, 40.0f, 1.5f),
                        Vector(175.5f, -10.0f, 1.5f),
                        Vector(75.5f, 25.0f, 1.5f),
                        Vector(80.5f, 45.0f, 1.5f),
                        Vector(140.5f, -41.0f, 1.5f),
                        Vector(45.5f, -38.0f, 1.5f),
                        Vector(85.5f, -40.0f, 1.5f),
                        Vector(-50.5f, -130.0f, 1.5f),
                        Vector(-46.5f, -20.0f, 1.5f),
                        Vector(-20.0f, -160.0f, 1.5f),
                        Vector(7.5f, -163.2f, 1.5f),
                        Vector(35.5f, -159.0f, 1.5f),
                        Vector(100.5f, -160.0f, 1.5f),
                        Vector(-5.0f, 7.0f, 1.5f),
                        Vector(7.0f, 40.0f, 1.5f),
                        Vector(-7.0f, -85.0f, 1.5f),
                        Vector(-107.0f, -81.5f, 1.5f),
                        Vector(-67.0f, -84.0f, 1.5f),
                        Vector(-67.0f, -29.0f, 1.5f),
                        Vector(-107.0f, -25.0f, 1.5f),
                        Vector(-100.0f, -124.0f, 1.5f),
                    };

                    // Place coins
                    for (const auto& pos : coinPositions) {
                        
                        placeCoins(pos.x, pos.y, pos.z);
                    }

                    // Start the timer
                    remainingTime = 10.0f;
                    timerActive = true;
                }

                // Reset ball back to original position and orientation
                if (ImGui::Button("Reset")) {
                    if (soccerBall != nullptr && soccerBallActor != nullptr) {
                        this->worldLst->eraseViaWOptr(soccerBall);
                        scene->removeActor(*soccerBallActor);
                        delete soccerBall;
                        soccerBall = nullptr;
                        soccerBallActor->release();
                        soccerBallActor = nullptr;
                    }
                    w = false;
                    a = false;
                    s = false;
                    d = false;
                    space = false;
                    soccerBall = WO::New(footballString, Vector(.1, .1, .1), MESH_SHADING_TYPE::mstSMOOTH);
                    soccerBall->setPosition(25, 5, 2.5);
                    soccerBall->setLabel("soccerBall");
                    soccerBall->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
                    this->worldLst->push_back(soccerBall);
                    gMaterial = p->createMaterial(0.5f, 0.55f, 0.3f);
                    soccerBallShape = p->createShape(physx::PxBoxGeometry(1.1, 1.1, 1.1), *gMaterial);
                    physx::PxTransform t({ 25, -2.5, 2.5 });
                    soccerBallActor = p->createRigidDynamic(t);
                    soccerBallActor->attachShape(*soccerBallShape);
                    scene->addActor(*soccerBallActor);
                }
                ImGui::End();
            };
        });
    this->worldLst->push_back(gui);
    createFinalProjectWayPoints();
    
}

void GLViewFinalProject::createFinalProjectWayPoints()
{
    // Create a waypoint with a radius of 3, a frequency of 5 seconds, activated by GLView's camera, and is visible.
    WayPointParametersBase params(this);
    params.frequency = 5000;
    params.useCamera = true;
    params.visible = true;
    WOWayPointSpherical* wayPt = WOWayPointSpherical::New(params, 3);
    wayPt->setPosition(Vector(50, 0, 3));
    worldLst->push_back(wayPt);
}