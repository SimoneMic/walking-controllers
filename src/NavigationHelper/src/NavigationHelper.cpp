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

#include <WalkingControllers/YarpUtilities/Helper.h>


using namespace WalkingControllers;

NavigationHelper::NavigationHelper()
{
}
    
NavigationHelper::~NavigationHelper()
{
}

bool NavigationHelper::setThreads(bool status)
{
    m_runThreads = status;
    return true;
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

bool NavigationHelper::init(const yarp::os::Searchable& config, std::deque<bool> &leftInContact, std::deque<bool> &rightInContact,
                        std::unique_ptr<WalkingFK> &FKSolver, 
                        std::unique_ptr<StableDCMModel> &stableDCMModel, 
                        std::unique_ptr<TrajectoryGenerator> &trajectoryGenerator)
{
    m_leftInContact = &leftInContact;
    m_rightInContact = &rightInContact;
    //ports for navigation integration
    std::string unicyclePortPortName = "/virtual_unicycle_states:o";
    if (!m_unicyclePort.open(unicyclePortPortName))
    {
        yError() << "[WalkingModule::configure] Could not open virtual_unicycle_states port";
        return false;
    }

    std::string replanningTriggerPortPortName = "/replanning_trigger:o";
    if (!m_replanningTriggerPort.open(replanningTriggerPortPortName))
    {
        yError() << "[NavigationHelper::init] Could not open" << replanningTriggerPortPortName << " port.";
        return false;
    }

    std::string feetPositionsPortPortName = "/feet_positions:o";
    if (!m_feetPort.open(feetPositionsPortPortName))
    {
        yError() << "[WalkingModule::configure] Could not open feet_positions port";
    }

    yarp::os::Bottle& trajectoryPlannerOptions = config.findGroup("NAVIGATION");
    m_navigationReplanningDelay = trajectoryPlannerOptions.check("navigationReplanningDelay", yarp::os::Value(0.9)).asFloat64();
    m_navigationTriggerLoopRate = trajectoryPlannerOptions.check("navigationTriggerLoopRate", yarp::os::Value(100)).asInt32();
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

    m_runThreads = true;
    m_virtualUnicyclePubliserThread = std::thread(&NavigationHelper::computeVirtualUnicycleThread, this, std::ref(FKSolver), std::ref(stableDCMModel), std::ref(trajectoryGenerator));
    m_navigationTriggerThread = std::thread(&NavigationHelper::computeNavigationTrigger, this);
    return true;
}

// This thread synchronizes the walking-controller with the navigation stack.
// Writes on a port a boolean value when to replan the path
void NavigationHelper::computeNavigationTrigger()
{
    bool trigger = false;   //flag used to fire the wait for sending the navigation replanning trigger
    yInfo() << "[NavigationHelper::computeNavigationTrigger] Starting Thread";
    yarp::os::NetworkClock myClock;
    myClock.open("/clock", "/navigationTriggerClock");
    bool enteredDoubleSupport = false, exitDoubleSupport = true;
    while (m_runThreads)
    {
        // Block the thread until the robot is in the walking state
        if (m_leftInContact == nullptr || m_rightInContact == nullptr)
        {
            //kill thread
            yError() << "[NavigationHelper::computeNavigationTrigger] null pointer to m_leftInContact or m_rightInContact. Killing thread.";
            return;
        }

        //double support check
        if (m_leftInContact->size()>0 && m_rightInContact->size()>0)  //external consistency check
        {
            if (m_leftInContact->at(0) && m_rightInContact->at(0))
            {
                if (exitDoubleSupport)
                {
                    enteredDoubleSupport = true;
                    exitDoubleSupport = false;
                }
            }
            else
            {
                if (! exitDoubleSupport)
                {
                    trigger = true; //in this way we have only one trigger each exit of double support
                }
                exitDoubleSupport = true;
            }
        }
        else
        {
            trigger = false;
            yWarning() << "[NavigationHelper::computeNavigationTrigger] one of the feet deques is empty" ;
        }
        

        //send the replanning trigger after a certain amount of seconds
        if (trigger)
        {
            trigger = false;
            //waiting -> could make it dependant by the current swing step duration
            myClock.delay(m_navigationReplanningDelay);
            yDebug() << "[NavigationHelper::computeNavigationTrigger] Triggering navigation replanning";
            auto& b = m_replanningTriggerPort.prepare();
            b.clear();
            b.add((yarp::os::Value)true);   //send the planning trigger
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
        if (m_leftInContact->size() > 0)
        {
            if (m_leftInContact->at(0))
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
        else if (m_rightInContact->size() > 0)
        {
            if (m_rightInContact->at(0))
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
            if (trajectoryGenerator->getUnicycleState(virtualUnicyclePose, virtualUnicycleReference, stanceFoot))
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
    if (m_feetPort.isClosed())
    {
        yError() << "[NavigationHelper::publishPlannedFootsteps] Feet port closed (no data sent).";
        return false;
    }
    
    //Send footsteps info on port anyway (x, y, yaw) wrt root_link
    std::vector<iDynTree::Vector3> leftFootprints, rightFootprints;
    if (trajectoryGenerator->getFootprints(leftFootprints, rightFootprints))
    {
        if (leftFootprints.size()>0 && rightFootprints.size()>0)
        {
            auto& feetData = m_feetPort.prepare();
            feetData.clear();
            auto& rightFeet = feetData.addList();
            auto& leftFeet = feetData.addList();
            //left foot
            for (size_t i = 0; i < leftFootprints.size(); ++i)
            {
                auto& pose = leftFeet.addList();
                pose.addFloat64(leftFootprints[i](0));   //x
                pose.addFloat64(leftFootprints[i](1));   //y
                pose.addFloat64(leftFootprints[i](2));   //yaw
            }
            //right foot
            for (size_t j = 0; j < rightFootprints.size(); ++j)
            {
                auto& pose = rightFeet.addList();
                pose.addFloat64(rightFootprints[j](0));   //x
                pose.addFloat64(rightFootprints[j](1));   //y
                pose.addFloat64(rightFootprints[j](2));   //yaw
            }
            m_feetPort.write();
        }
    }
    return true;
}