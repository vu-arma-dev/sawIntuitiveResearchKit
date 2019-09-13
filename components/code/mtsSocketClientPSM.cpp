/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Pretham Chalasani, Anton Deguet
  Created on: 2016-11-04

  (C) Copyright 2016-2018 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <sawIntuitiveResearchKit/mtsSocketClientPSM.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>


CMN_IMPLEMENT_SERVICES_DERIVED(mtsSocketClientPSM, mtsTaskPeriodic);

mtsSocketClientPSM::mtsSocketClientPSM(const std::string & componentName, const double periodInSeconds,
                                       const std::string & ip, const unsigned int port) :
    mtsSocketBasePSM(componentName, periodInSeconds, ip, port, false)
{
    this->StateTable.AddData(PositionCartesianCurrent, "PositionCartesianCurrent");
    StateJaw.Position().resize(1);
    this->StateTable.AddData(StateJaw, "StateJaw");
    this->StateTable.AddData(ForceCartesianCurrent, "ForceCartesianCurrent");
    this->StateTable.AddData(ForceCartesianCurrentGT, "ForceCartesianCurrentGT");
    this->StateTable.AddData(MagCartesianCurrent, "MagCartesianCurrent");
    this->StateTable.AddData(MagVecCurrent, "MagVecCurrent");

    mtsInterfaceProvided * interfaceProvided = AddInterfaceProvided("Robot");
    if (interfaceProvided) {
        interfaceProvided->AddMessageEvents();
        interfaceProvided->AddCommandReadState(this->StateTable, PositionCartesianCurrent, "GetPositionCartesian");
        interfaceProvided->AddCommandReadState(this->StateTable, StateJaw, "GetStateJaw");
        interfaceProvided->AddCommandReadState(this->StateTable, ForceCartesianCurrent, "GetForceCartesian");
        interfaceProvided->AddCommandReadState(this->StateTable, ForceCartesianCurrentGT, "GetForceCartesianGT");
        interfaceProvided->AddCommandReadState(this->StateTable, MagCartesianCurrent, "GetMagCartesianCurrent");
        interfaceProvided->AddCommandReadState(this->StateTable, MagVecCurrent, "GetMagVecCurrent");

        interfaceProvided->AddCommandVoid(&mtsSocketClientPSM::Freeze,
                                          this, "Freeze");
        interfaceProvided->AddCommandWrite(&mtsSocketClientPSM::SetPositionCartesian,
                                           this , "SetPositionCartesian");
        interfaceProvided->AddCommandWrite(&mtsSocketClientPSM::SetPositionJaw,
                                           this , "SetPositionJaw");
        interfaceProvided->AddCommandWrite(&mtsSocketClientPSM::SetDesiredState,
                                           this , "SetDesiredState");
        interfaceProvided->AddCommandWrite(&mtsSocketClientPSM::SetForceGain,
                                           this , "SetForceGain");
        interfaceProvided->AddCommandRead(&mtsSocketClientPSM::GetDesiredState,
                                           this , "GetDesiredState");
        interfaceProvided->AddCommandRead(&mtsSocketClientPSM::GetCurrentState,
                                           this , "GetCurrentState");

        interfaceProvided->AddCommandWrite(&mtsSocketClientPSM::SetDesiredSpecial,
                                           this , "SetDesiredSpecial");
        interfaceProvided->AddCommandRead(&mtsSocketClientPSM::GetDesiredSpecial,
                                           this , "GetDesiredSpecial");
        interfaceProvided->AddCommandRead(&mtsSocketClientPSM::GetCurrentSpecial,
                                           this , "GetCurrentSpecial");

    }
}

void mtsSocketClientPSM::Configure(const std::string & CMN_UNUSED(fileName))
{
    DesiredState = socketMessages::SCK_UNINITIALIZED;
    CurrentState = socketMessages::SCK_UNINITIALIZED;
    Command.Data.Header.Size = CLIENT_MSG_SIZE_CMD;
    Command.Socket->SetDestination(IpAddress, Command.IpPort);
    State.Socket->AssignPort(State.IpPort);
    forceEstimateGain.Assign(1,1,1);
}

void mtsSocketClientPSM::Run(void)
{
    ProcessQueuedEvents();
    ProcessQueuedCommands();

    ReceivePSMStateData();
    UpdateStatistics();
    SendPSMCommandData();
}

void mtsSocketClientPSM::UpdateApplication(void)
{
    CurrentState = State.Data.RobotControlState;
    PositionCartesianCurrent.Valid() = (CurrentState >= socketMessages::SCK_HOMED);
    PositionCartesianCurrent.Position().FromNormalized(State.Data.CurrentPose);
    vct3 forceCurrent(State.Data.CurrentForce.at(0)*forceEstimateGain.at(0),
                      State.Data.CurrentForce.at(1)*forceEstimateGain.at(1),
                      State.Data.CurrentForce.at(2)*forceEstimateGain.at(2));

    //TODO: change moving average to live in a function
    //TODO: Make moving average size editable
    forceList.push_front(forceCurrent);
    if (forceList.size()>1)
        forceList.pop_back();
    vct3 forceSum(0,0,0);
    for (int ii=0;ii<forceList.size();ii++)
    {
        forceSum.at(0) = forceSum.at(0) + forceList.at(ii).at(0);
        forceSum.at(1) = forceSum.at(1) + forceList.at(ii).at(1);
        forceSum.at(2) = forceSum.at(2) + forceList.at(ii).at(2);
    }

    ForceCartesianCurrent.Force().Assign(forceSum.at(0)/forceList.size(),
                                         forceSum.at(1)/forceList.size(),
                                         forceSum.at(2)/forceList.size(),
                                         0, 0, 0);
    ForceCartesianCurrent.Valid() = true;

    

    forceCurrent.at(0)=State.Data.CurrentForceGT.at(0);
    forceCurrent.at(1)=State.Data.CurrentForceGT.at(1);
    forceCurrent.at(2)=State.Data.CurrentForceGT.at(2);

    forceListGT.push_front(forceCurrent);
    if (forceListGT.size()>5)
        forceListGT.pop_back();

    forceSum.SetAll(0);
    for (int ii=0;ii<forceListGT.size();ii++)
    {
        forceSum.at(0) = forceSum.at(0) + forceListGT.at(ii).at(0);
        forceSum.at(1) = forceSum.at(1) + forceListGT.at(ii).at(1);
        forceSum.at(2) = forceSum.at(2) + forceListGT.at(ii).at(2);
    }
    ForceCartesianCurrentGT.Force().Assign(forceSum.at(0)/forceListGT.size(),
                                          forceSum.at(1)/forceListGT.size(),
                                          forceSum.at(2)/forceListGT.size(),
                                          0, 0, 0);
    ForceCartesianCurrentGT.Valid() = true;
    StateJaw.Position().at(0) = State.Data.CurrentJaw;
    StateJaw.Valid() = true;

    MagCartesianCurrent = State.Data.CurrentMagPos;
    MagVecCurrent = State.Data.CurrentMagVec;

}

void mtsSocketClientPSM::Freeze(void)
{
    DesiredState = socketMessages::SCK_CART_POS;
    Command.Data.GoalPose.From(State.Data.CurrentPose);
    Command.Data.GoalJaw = State.Data.CurrentJaw;
}

void mtsSocketClientPSM::SetDesiredState(const std::string & state)
{
    if (state == "UNINITIALIZED") {
        DesiredState = socketMessages::SCK_UNINITIALIZED;
    } else if (state == "READY") {
        DesiredState = socketMessages::SCK_HOMED;
    } else {
        std::cerr << CMN_LOG_DETAILS << state << " state not supported." << std::endl;
    }

    Command.Data.RobotControlState = DesiredState;
    Command.Data.GoalPose.From(State.Data.CurrentPose);
    Command.Data.GoalJaw = State.Data.CurrentJaw;
}

void mtsSocketClientPSM::SetDesiredSpecial(const std::string & special)
{
    std::cerr << "Setting Special: " << special << std::endl;
    if (special == "HFC_ON") {
        DesiredSpecial = socketMessages::HFC_ON;
    } else if (special == "HFC_OFF") {
        DesiredSpecial = socketMessages::HFC_OFF;
    } else {
        std::cerr << CMN_LOG_DETAILS << special << " state not supported." << std::endl;
    }

    // Command.Data.RobotControlState = DesiredState;
    // Command.Data.GoalPose.From(State.Data.CurrentPose);
    // Command.Data.GoalJaw = State.Data.CurrentJaw;
    Command.Data.GoalSpecial = DesiredSpecial;
}

void mtsSocketClientPSM::SetPositionCartesian(const prmPositionCartesianSet & position)
{
    DesiredState = socketMessages::SCK_CART_POS;
    Command.Data.GoalPose.From(position.Goal());
}

void mtsSocketClientPSM::SetForceGain(const vct3 & forceGainInput)
{
    forceEstimateGain = forceGainInput;
}

void mtsSocketClientPSM::SetPositionJaw(const prmPositionJointSet & position)
{
    DesiredState = socketMessages::SCK_CART_POS;
    Command.Data.GoalJaw = position.Goal().at(0);
}

void mtsSocketClientPSM::GetDesiredState(std::string & state) const
{
    switch (DesiredState) {
    case socketMessages::SCK_UNINITIALIZED:
        state = "UNINITIALIZED";
        break;
    case socketMessages::SCK_HOMING:
    case socketMessages::SCK_HOMED:
    case socketMessages::SCK_CART_POS:
        state = "READY";
        break;
    default:
        std::cerr << CMN_LOG_DETAILS << DesiredState << " state not supported." << std::endl;
        break;
    }
}

void mtsSocketClientPSM::GetDesiredSpecial(std::string & special) const
{
    switch (DesiredSpecial) {
    case socketMessages::HFC_OFF:
        special = "HFC_OFF";
        break;
    case socketMessages::HFC_ON:
        special = "HFC_ON";
        break;
    default:
        std::cerr << CMN_LOG_DETAILS << special << " command not supported." << std::endl;
        break;
    }
}

void mtsSocketClientPSM::GetCurrentState(std::string & state) const
{
    switch (CurrentState) {
    case socketMessages::SCK_UNINITIALIZED:
        state = "UNINITIALIZED";
        break;
    case socketMessages::SCK_HOMING:
    case socketMessages::SCK_HOMED:
    case socketMessages::SCK_CART_POS:
        state = "READY";
        break;
    default:
        std::cerr << CMN_LOG_DETAILS << state << " state not supported." << std::endl;
        break;
    }
}

void mtsSocketClientPSM::GetCurrentSpecial(std::string & special) const
{
    switch (CurrentSpecial) {
    case socketMessages::HFC_OFF:
        special = "HFC_OFF";
        break;
    case socketMessages::HFC_ON:
        special = "HFC_ON";
        break;
    default:
        std::cerr << CMN_LOG_DETAILS << CurrentSpecial << " command not supported." << std::endl;
        break;
    }
}

void mtsSocketClientPSM::ReceivePSMStateData(void)
{
    // Recv Scoket Data
    size_t bytesRead = 0;
    bytesRead = State.Socket->Receive(State.Buffer, BUFFER_SIZE, TIMEOUT);
    if (bytesRead > 0) {
        if (bytesRead != State.Data.Header.Size){
            std::cerr << CMN_LOG_DETAILS << "Incorrect bytes read " << bytesRead << ". Looking for " << State.Data.Header.Size << " bytes." << std::endl;
        }

        std::stringstream ss;
        cmnDataFormat local, remote;
        ss.write(State.Buffer, bytesRead);

        // Dequeue all the datagrams and only use the latest one.
        int readCounter = 0;
        int dataLeft = bytesRead;
        while (dataLeft > 0) {
            dataLeft = State.Socket->Receive(State.Buffer, BUFFER_SIZE, 0);
            if (dataLeft != 0) {
                bytesRead = dataLeft;
            }

            readCounter++;
        }

        if (readCounter > 1) {
            std::cerr << CMN_LOG_DETAILS << "Catching up : " << readCounter << std::endl;
        }

        ss.write(State.Buffer, bytesRead);
        cmnData<socketStatePSM>::DeSerializeBinary(State.Data, ss, local, remote);

        State.Data.CurrentPose.NormalizedSelf();
        UpdateApplication();
    } else {
        CMN_LOG_CLASS_RUN_DEBUG << "RecvPSMStateData: UDP receive failed" << std::endl;
    }
}

void mtsSocketClientPSM::SendPSMCommandData(void)
{
    // Update Header
    Command.Data.Header.Id++;
    Command.Data.Header.Timestamp = mTimeServer.GetRelativeTime();
    Command.Data.Header.LastId = State.Data.Header.Id;
    Command.Data.Header.LastTimestamp = State.Data.Header.Timestamp;
    Command.Data.RobotControlState = DesiredState;

    // Send Socket Data
    std::stringstream ss;
    cmnData<socketCommandPSM>::SerializeBinary(Command.Data, ss);
    memcpy(Command.Buffer, ss.str().c_str(), ss.str().length());

    Command.Socket->Send(Command.Buffer, ss.str().size());
}
