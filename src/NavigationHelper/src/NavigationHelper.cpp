/**
 * @file NavigationHelper.cpp
 * @authors Simone Micheletti <simone.micheletti@iit.it>
 * @copyright 2023 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2023
 */

#include <iostream>
#include <thread>
#include <WalkingControllers/NavigationHelper/NavigationHelper.h>

#include <yarp/os/Stamp.h>
#include <yarp/os/NetworkClock.h>
#include <yarp/os/Contact.h>

#include <WalkingControllers/YarpUtilities/Helper.h>
#include <WalkingControllers/TrajectoryPlanner/TrajectoryGenerator.h>


using namespace WalkingControllers;

NavigationHelper::NavigationHelper()
{
    m_wasInDoubleSupport = false;
}

NavigationHelper::~NavigationHelper()
{
}

bool NavigationHelper::closeThreads()
{
    //Close parallel threads
    if (m_runThreads)
    {
        m_runThreads = false;
        yInfo() << "Closing m_virtualUnicyclePubliserThread";
        if(m_virtualUnicyclePubliserThread.joinable())
        {
            m_virtualUnicyclePubliserThread.join();
            m_virtualUnicyclePubliserThread = std::thread();
        }

        yInfo() << "Closing m_navigationTriggerThread";
        if(m_navigationTriggerThread.joinable())
        {
            m_navigationTriggerThread.join();
            m_navigationTriggerThread = std::thread();
        }
    }
    return true;
}

bool NavigationHelper::closeHelper()
{
    try
    {
        //close threads first
        if (m_runThreads)
        {
            closeThreads();
        }
        //close ports
        if(!m_unicyclePort.isClosed())
            m_unicyclePort.close();
        if(!m_replanningTriggerPort.isClosed())
            m_replanningTriggerPort.close();
        if(!m_feetPort.isClosed())
            m_feetPort.close();

    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return false;
    }
    return true;
}

bool NavigationHelper::init(const yarp::os::Searchable& config, 
                        std::unique_ptr<WalkingFK> &FKSolver, 
                        std::unique_ptr<StableDCMModel> &stableDCMModel, 
                        std::unique_ptr<TrajectoryGenerator> &trajectoryGenerator)
{
    m_navigationReplanningDelay = config.check("navigationReplanningDelay", yarp::os::Value(0.9)).asFloat64();
    m_navigationTriggerLoopRate = config.check("navigationTriggerLoopRate", yarp::os::Value(100)).asInt32();
    m_publishInfo = config.check("publishNavigationInfo", yarp::os::Value(false)).asBool();
    m_simulationMode = config.check("simulationMode", yarp::os::Value(false)).asBool();
    if (config.check("plannerMode", yarp::os::Value("manual")).asString() == "navigation")
    {
        //if in navigation mode we need this parameter to be true
        m_publishInfo = true;
    }
    if (!m_publishInfo)
    {   //exit the funnction if the infos are not needed
        yInfo() << "[NavigationHelper::init] Configuring NavigationHelper without publishing infos on ports ";
        return true;
    }

    // format check
    if (m_navigationTriggerLoopRate<=0)
    {
        yError() << "[configure] navigationTriggerLoopRate must be strictly positive, instead is: " << m_navigationTriggerLoopRate;
        return false;
    }
    if (m_navigationReplanningDelay<0)
    {
        yError() << "[configure] navigationTriggerLoopRate must be positive, instead is: " << m_navigationReplanningDelay;
        return false;
    }
    
    //ports for navigation integration
    std::string unicyclePortPortName = m_portPrefix + "/virtual_unicycle_states:o";
    yarp::os::Contact unicyclePortContact{unicyclePortPortName, "shmem" };
    if (!m_unicyclePort.open(unicyclePortContact, true))
    {
        yError() << "[WalkingModule::configure] Could not open virtual_unicycle_states port";
        return false;
    }

    std::string replanningTriggerPortPortName = m_portPrefix + "/replanning_trigger:o";
    if (!m_replanningTriggerPort.open(replanningTriggerPortPortName))
    {
        yError() << "[NavigationHelper::init] Could not open" << replanningTriggerPortPortName << " port.";
        return false;
    }

    std::string feetPositionsPortPortName = m_portPrefix + "/feet_positions:o";
    if (!m_feetPort.open(feetPositionsPortPortName))
    {
        yError() << "[WalkingModule::configure] Could not open feet_positions port";
        return false;
    }

    m_runThreads = true;
    m_virtualUnicyclePubliserThread = std::thread(&NavigationHelper::computeVirtualUnicycleThread, this, std::ref(FKSolver), std::ref(stableDCMModel), std::ref(trajectoryGenerator));
    m_navigationTriggerThread = std::thread(&NavigationHelper::computeNavigationTrigger, this);
    return true;
}

// This thread synchronizes the walking-controller with the navigation stack.
// Writes on a port a boolean value when to replan the path
void NavigationHelper::computeNavigationTrigger()
{
    if (!m_publishInfo)
    {   //exit the function if the infos are not needed
        return;
    }
    bool trigger = false;   //flag used to fire the wait for sending the navigation replanning trigger
    yInfo() << "[NavigationHelper::computeNavigationTrigger] Starting Thread";
    yarp::os::NetworkClock myClock;
    if (m_simulationMode)
    {
        myClock.open("/clock", "/navigationTriggerClock");
    }
    bool enteredDoubleSupport = false, exitDoubleSupport = true;
    while (m_runThreads)
    {
        {
            std::unique_lock<std::mutex> lock(m_updateFeetMutex);
            if (m_leftInContact.size()>0 && m_rightInContact.size()>0)  //external consistency check
            {
                if (m_leftInContact.at(0) && m_rightInContact.at(0))    //double support check
                {
                    if (exitDoubleSupport)
                    {
                        enteredDoubleSupport = true;
                        exitDoubleSupport = false;
                    }
                }
                else
                {
                    if (!exitDoubleSupport)
                    {
                        trigger = true; //in this way we have only one trigger each exit of double support
                    }
                    exitDoubleSupport = true;
                }
            }
            else
            {
                trigger = false;
            }
        }
        
        //send the replanning trigger after a certain amount of seconds
        if (trigger)
        {
            trigger = false;
            //waiting -> could make it dependant by the current swing step duration
            //if in simulation we need to sync the clock, we can't use the system clock
            if (m_simulationMode)
            {
                myClock.delay(m_navigationReplanningDelay);
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(m_navigationReplanningDelay*1000));
            }
            yInfo() << "[NavigationHelper::computeNavigationTrigger] Triggering navigation replanning";
            auto& b = m_replanningTriggerPort.prepare();
            b.clear();
            b.add((yarp::os::Value)true);   //send the planning trigger
            //Timestamp
            yarp::os::Stamp stamp(0, yarp::os::Time::now());
            m_replanningTriggerPort.setEnvelope(stamp);
            m_replanningTriggerPort.write();
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1000/m_navigationTriggerLoopRate));
    }
    yInfo() << "[NavigationHelper::computeNavigationTrigger] Terminated thread";
}

// This thread publishes the internal informations of the virtual unicycle extracted from the feet and the CoM speed
// It's the internal odometry
void NavigationHelper::computeVirtualUnicycleThread(std::unique_ptr<WalkingFK> &FKSolver, 
                                                    std::unique_ptr<StableDCMModel> &stableDCMModel, 
                                                    std::unique_ptr<TrajectoryGenerator> &trajectoryGenerator
                                                    )
{
    if (!m_publishInfo)
    {   //exit the funnction if the infos are not needed
        return;
    }
    
    yInfo() << "[NavigationHelper::computeVirtualUnicycleThread] Starting Thread";
    bool inContact = false;
    while (m_runThreads)
    {
        //if (m_robotState != 4)
        //{
        //    std::this_thread::sleep_for(std::chrono::milliseconds(1000/m_navigationTriggerLoopRate));
        //    continue;
        //}

        iDynTree::Vector3 virtualUnicyclePose, virtualUnicycleReference;
        std::string stanceFoot;
        iDynTree::Transform footTransformToWorld, root_linkTransform;
        //Stance foot check
        if (m_leftInContact.size() > 0)
        {
            if (m_leftInContact.at(0))
            {
                stanceFoot = "left";
                footTransformToWorld = FKSolver->getLeftFootToWorldTransform();
            }
            else
            {
                stanceFoot = "right";
                footTransformToWorld = FKSolver->getRightFootToWorldTransform();
            }
            inContact = true;
        }
        else if (m_rightInContact.size() > 0)
        {
            if (m_rightInContact.at(0))
            {
                stanceFoot = "right";
                footTransformToWorld = FKSolver->getRightFootToWorldTransform();
            }
            else
            {
                stanceFoot = "left";
                footTransformToWorld = FKSolver->getLeftFootToWorldTransform();
            }
            inContact = true;
        }
        else
        {
            inContact = false;
        }
        root_linkTransform = FKSolver->getRootLinkToWorldTransform();   //world -> rootLink transform
        //Data conversion and port writing
        if (inContact)
        {
            if (trajectoryGenerator->getUnicycleState(stanceFoot, virtualUnicyclePose, virtualUnicycleReference))
            {
                //send data
                yarp::os::Stamp stamp (0, yarp::os::Time::now());   //move to private member of the class
                m_unicyclePort.setEnvelope(stamp);
                auto& data = m_unicyclePort.prepare();
                data.clear();
                auto& unicyclePose = data.addList();
                unicyclePose.addFloat64(virtualUnicyclePose(0));
                unicyclePose.addFloat64(virtualUnicyclePose(1));
                unicyclePose.addFloat64(virtualUnicyclePose(2));
                auto& referencePose = data.addList();
                referencePose.addFloat64(virtualUnicycleReference(0));
                referencePose.addFloat64(virtualUnicycleReference(1));
                referencePose.addFloat64(virtualUnicycleReference(2));
                data.addString(stanceFoot);

                //add root link stransform: X, Y, Z, r, p, yaw,
                auto& transform = data.addList();
                transform.addFloat64(root_linkTransform.getPosition()(0));
                transform.addFloat64(root_linkTransform.getPosition()(1));
                transform.addFloat64(root_linkTransform.getPosition()(2));
                transform.addFloat64(root_linkTransform.getRotation().asRPY()(0));
                transform.addFloat64(root_linkTransform.getRotation().asRPY()(1));
                transform.addFloat64(root_linkTransform.getRotation().asRPY()(2));

                //planned velocity of the CoM
                auto comVel = stableDCMModel->getCoMVelocity();
                auto& velData = data.addList();
                velData.addFloat64(comVel(0));
                velData.addFloat64(comVel(1));
                //TODO add angular velocity
                velData.addFloat64(0.0);

                //measured velocity of the root link
                auto measuredVel = FKSolver->getRootLinkVelocity();
                auto& measuredVelData = data.addList();
                measuredVelData.addFloat64(measuredVel(0));
                measuredVelData.addFloat64(measuredVel(1));
                measuredVelData.addFloat64(measuredVel(2));
                measuredVelData.addFloat64(measuredVel(3));
                measuredVelData.addFloat64(measuredVel(4));
                measuredVelData.addFloat64(measuredVel(5));

                m_unicyclePort.write();
            }
            else
            {
                yError() << "[NavigationHelper::computeVirtualUnicycleThread] Could not getUnicycleState from m_trajectoryGenerator (no data sent).";
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000/m_navigationTriggerLoopRate));
    }
    yInfo() << "[NavigationHelper::computeVirtualUnicycleThread] Terminated thread";
}

bool NavigationHelper::publishPlannedFootsteps(std::unique_ptr<TrajectoryGenerator> &trajectoryGenerator)
{
    if (!m_publishInfo)
    {   //exit the funnction if the infos are not needed
        return true;
    }
    if (m_feetPort.isClosed())
    {
        yError() << "[NavigationHelper::publishPlannedFootsteps] Feet port closed (no data sent).";
        return false;
    }
    
    //Send footsteps info on port anyway (x, y, yaw) wrt root_link
    std::vector<TrajectoryGenerator::Pose> leftFootprints, rightFootprints;
    if (trajectoryGenerator->getFootprints(leftFootprints, rightFootprints))
    {
        if (leftFootprints.size()>0 && rightFootprints.size()>0)
        {
            auto& feetData = m_feetPort.prepare();
            feetData.clear();
            auto& rightFeet = feetData.addList();
            auto& leftFeet = feetData.addList();
            //Timestamp
            yarp::os::Stamp stamp(0, yarp::os::Time::now());
            m_feetPort.setEnvelope(stamp);
            //left foot
            for (size_t i = 0; i < leftFootprints.size(); ++i)
            {
                auto& pose = leftFeet.addList();
                pose.addFloat64(leftFootprints[i].x);   //x
                pose.addFloat64(leftFootprints[i].y);   //y
                pose.addFloat64(leftFootprints[i].theta);   //yaw
            }
            //right foot
            for (size_t j = 0; j < rightFootprints.size(); ++j)
            {
                auto& pose = rightFeet.addList();
                pose.addFloat64(rightFootprints[j].x);   //x
                pose.addFloat64(rightFootprints[j].y);   //y
                pose.addFloat64(rightFootprints[j].theta);   //yaw
            }
            m_feetPort.write();
        }
    }
    return true;
}

bool NavigationHelper::updateFeetDeques(const std::deque<bool> &left, const std::deque<bool> &right)
{
    std::unique_lock<std::mutex> lock(m_updateFeetMutex);
    m_leftInContact = left;
    m_rightInContact = right;
    return true;
}
