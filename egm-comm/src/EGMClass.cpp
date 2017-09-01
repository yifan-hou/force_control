#include <iostream>
#include <cmath>
#include "EGMClass.h"

#define RCBFLENGTH 1400
// #define PI 3.1416  // already defined in matVec

using namespace std;

EGMClass* EGMClass::pinstance = 0;


void copyArray(float *src, float *dest, int dim)
{
    for(int i = 0; i<dim; i++)
    {
        dest[i] = src[i];
    }
}

void setArray(float *array, float value, int dim)
{
    for(int i=0; i<dim; i++)
    {
        array[i] = value;
    }
}

//http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/
Quaternion SlerpFixAngle(Quaternion &qa, Quaternion &qb, float angle)
{
    // quaternion to return
    Quaternion qm;
    // Calculate angle between them.
    double cosHalfTheta = qa.w() * qb.w() + qa.x() * qb.x() + qa.y() * qb.y() + qa.z() * qb.z();
    // if qa=qb or qa=-qb then theta = 0 and we can return qa
    if (abs(cosHalfTheta) >= 1.0){
        qm.w() = qa.w(); qm.x() = qa.x(); qm.y() = qa.y();qm.z() = qa.z();
        return qm;
    }
    // Calculate temporary values. acos return [0, PI]
    double halfTheta = acos(cosHalfTheta);

    // qa-qb is smaller than angle. Return qb
    if (2*halfTheta < angle)
    {
        qm.w() = qb.w(); qm.x() = qb.x(); qm.y() = qb.y(); qm.z() = qb.z();
        return qm;
    }

    double sinHalfTheta = sqrt(1.0 - cosHalfTheta*cosHalfTheta);
    // if theta = 180 degrees then result is not fully defined
    // do not move
    if (fabs(sinHalfTheta) < 0.001){ 
        qm.w() = qa.w(); qm.x() = qa.x(); qm.y() = qa.y();qm.z() = qa.z();
        return qm;
    }

    angle /=2;
    double ratioA = sin(halfTheta-angle) / sinHalfTheta;
    double ratioB = sin(angle) / sinHalfTheta; 

    //calculate Quaternion.
    qm.w() = (qa.w() * ratioA + qb.w() * ratioB);
    qm.x() = (qa.x() * ratioA + qb.x() * ratioB);
    qm.y() = (qa.y() * ratioA + qb.y() * ratioB);
    qm.z() = (qa.z() * ratioA + qb.z() * ratioB);
    return qm;
}

EGMClass* EGMClass::Instance()
{
	if (pinstance == 0) // first time call
	{
		pinstance = new EGMClass;
	}
	return pinstance;
}

EGMClass::EGMClass()
{
	// do some initialization
    _pose          = new float[7];
    _set_pose      = new float[7];
    _set_vel       = new float[6];
    _isInitialized = false;
    _RobotPort     = 0;
    _mode          = CT_MODE_POSITION;
    _vel_tran      = 0;
    _vel_rot       = 0;

}

EGMClass & EGMClass::operator=(const EGMClass & olc)
{
	// shall never get called
	EGMClass* lc = new EGMClass();
	return *lc;
}

EGMClass::~EGMClass()
{
	delete pinstance;
    delete [] _pose;
    delete [] _set_pose;
	delete [] _set_vel;
}

//
//  The thread Function.
//
void* Monitor(void* pParam)
{
    EGMClass *egm = (EGMClass*)pParam;
    static float pose[7], poseSet[7];
    egm->GetCartesian(pose);

    // -------------------------------------------
    //      send
    // -------------------------------------------

    if (egm->_mode == CT_MODE_POSITION)
    {
        static Vec tran(3);
        static Quaternion quatGoal, quatNow, quatSet;
        static float tranSet[3];

        float dist_limit_tran = egm->_vel_tran*EGM_PERIOD/1000.0;
        float dist_limit_rot  = egm->_vel_rot*EGM_PERIOD/1000.0;
        
        for(int i=0; i<3; i++) 
            tran[i] = egm->_set_pose[i] - pose[i];
        if (tran.norm() > dist_limit_tran) tran *= (dist_limit_tran/tran.norm());
        tranSet[0] = pose[0] + tran[0];
        tranSet[1] = pose[1] + tran[1];
        tranSet[2] = pose[2] + tran[2];

        for(int i = 0; i < 4; i++)
        {
            quatNow.v[i] = pose[i+3];
            quatGoal.v[i] = egm->_set_pose[i+3];
        }

        quatSet = SlerpFixAngle(quatNow, quatGoal, dist_limit_rot);

        copyArray(tranSet, poseSet, 3);
        quatSet.w() = poseSet[3];
        quatSet.x() = poseSet[4];
        quatSet.y() = poseSet[5];
        quatSet.z() = poseSet[6];
    }
    else if(egm->_mode == CT_MODE_VELOCITY)
    {
        copyArray(pose, poseSet, 7);
        poseSet[0] = pose[0] + egm->_set_vel[0]*EGM_PERIOD/1000.0;
        poseSet[1] = pose[1] + egm->_set_vel[1]*EGM_PERIOD/1000.0;
        poseSet[2] = pose[2] + egm->_set_vel[2]*EGM_PERIOD/1000.0;
    }

    egm->send(poseSet);

    // -------------------------------------------
    //      receiving
    // -------------------------------------------
    egm->listen();
}

int EGMClass::initialization(unsigned short portnum, float speed_limit_tran, float speed_limit_rot, ControlMode mode)
{
    /* Establish connection with ABB EGM */ 
    _EGMsock.setLocalPort(portnum);
    _isInitialized = true;
    cout << "EGM server is waiting for connection..\n";
    listen();
    _timer.tic();
    cout << "EGM connection established.\n";

    _mode = mode;
    _vel_limit_tran = speed_limit_tran;
    _vel_limit_rot  = speed_limit_rot;

    /* Initialize goal */
    copyArray(_pose, _set_pose, 7);
    setArray(_set_vel, 0, 6);

    /* Create thread to run communication with EGM */
    if (mode != CT_MODE_TRANSPARENT)
    {
        int rc = pthread_create(&_thread, NULL, Monitor, this);
        if (rc){
            cout <<"EGM error:unable to create thread.\n";
            return false;
        }
    }
    return true;
}

int EGMClass::GetCartesian(float *pose)
{
    if (!_isInitialized) return false;

    copyArray(pose, _pose, 7);
}

int EGMClass::SetCartesian(float *pose)
{
    if (_mode != CT_MODE_POSITION) return false;

    copyArray(pose, _set_pose, 7);

    return true;
}

int EGMClass::SetCartesianVel(float vel_tran, float vel_rot)
{
    if (_mode != CT_MODE_POSITION) return false;

    _vel_tran = vel_tran;
    _vel_rot  = vel_rot;

    return true;
}


int EGMClass::SetVelocity(float *v, float *w)
{
    if (_mode != CT_MODE_VELOCITY) return false;

    copyArray(v, _set_vel, 3);
    copyArray(w, _set_vel+3, 3);

    return true;
}

int EGMClass::listen()
{
    if (_isInitialized == false)
    {
        cout << "EGM error: listen() is called before initialization().\n";
        return false;
    }

	// cout << "If can not receive message, run 'sudo ufw allow 6510'\n";
    char recvBuffer[RCBFLENGTH];
	int n = _EGMsock.recvFrom(recvBuffer, RCBFLENGTH, _RobotAddress, _RobotPort);
	if (n < 0)
	{
	    cout << "EGM error: Error receiving message.\n";
	    exit(1);
	}
	// cout << "[Egm server node] message received, connection established!\n";
	// cout << "Address: " << _RobotAddress.c_str() << ", port:" << _RobotPort << endl;
	
	_pRecvMessage = new EgmRobot();
	_pRecvMessage->ParseFromArray(recvBuffer, n);
	ReadRobotMessage(_pRecvMessage);
	// printf("x: %lf\ny: %lf\nz: %lf\n", x, y, z);
	delete _pRecvMessage;

    return true;
}

void EGMClass::send(float *setpose)
{
    if (_RobotPort == 0)
    {
        cout << "EGM error: send() is called before listen().\n";
        return;
    }
	// -----------------------------------------------------------------------------
	//      create and send a sensor message
	// -----------------------------------------------------------------------------
	_pSendingMessage = new EgmSensor();
	if(!CreateSensorMessage(_pSendingMessage, setpose))
    {
        cout << "EGM error: the goal pos is too far away from current pos.\n";
        return;
    }
	_pSendingMessage->SerializeToString(&_sendBuffer);
	delete _pSendingMessage;

    _EGMsock.sendTo(_sendBuffer.c_str(), _sendBuffer.length(), _RobotAddress, _RobotPort);

}

int EGMClass::CreateSensorMessage(EgmSensor* pSensorMessage, float *setpose)
{
    // safety check: translation
    static Vec tran(3);
    tran[0] = setpose[0] - _pose[0];
    tran[1] = setpose[1] - _pose[1];
    tran[2] = setpose[2] - _pose[2];

    if (tran.norm() > _vel_limit_tran*EGM_PERIOD/1000.0) return false;

    // safety check: rotation
    // Calculate angle between them.
    double cosHalfTheta = _pose[3] * setpose[3] + _pose[4] * setpose[4] + _pose[5] * setpose[5] + _pose[6] * setpose[6];
    double theta = 2*acos(cosHalfTheta);
    if (theta > _vel_limit_rot*EGM_PERIOD/1000.0) return false;


    static unsigned int sequenceNumber = 0;
    EgmHeader* header = new EgmHeader();
    header->set_mtype(EgmHeader_MessageType_MSGTYPE_CORRECTION);
    header->set_seqno(sequenceNumber++);
    double time = _timer.toc();
    cout << "Message Created at time = " << time << "ms." << endl;
    header->set_tm(time);

    pSensorMessage->set_allocated_header(header);
    EgmCartesian *pc = new EgmCartesian();

    // in mm
    pc->set_x(setpose[0]);    
    pc->set_y(setpose[1]);
    pc->set_z(setpose[2]);
    
    EgmQuaternion *pq = new EgmQuaternion();
    pq->set_u0(setpose[3]);   
    pq->set_u1(setpose[4]);
    pq->set_u2(setpose[5]);
    pq->set_u3(setpose[6]);

    EgmPose *pcartesian = new EgmPose();
    pcartesian->set_allocated_orient(pq);
    pcartesian->set_allocated_pos(pc);

    EgmPlanned *planned = new EgmPlanned();
    planned->set_allocated_cartesian(pcartesian);

    pSensorMessage->set_allocated_planned(planned);

    return true;
}

// Create a simple robot message
void EGMClass::CreateSensorMessageEmpty(EgmSensor* pSensorMessage)
{
    static unsigned int sequenceNumber = 0;
    EgmHeader* header = new EgmHeader();
    header->set_mtype(EgmHeader_MessageType_MSGTYPE_CORRECTION);
    header->set_seqno(sequenceNumber++);
    header->set_tm(0);
    // header->set_tm(GetTickCount());

    pSensorMessage->set_allocated_header(header);

}

// ************************
void EGMClass::ReadRobotMessage(EgmRobot *pRobotMessage)
{
    if (pRobotMessage->has_header() && pRobotMessage->header().has_seqno() && pRobotMessage->header().has_tm() && pRobotMessage->header().has_mtype()  )
    {
        //printf("SeqNo=%d Tm=%u Type=%d\n", pRobotMessage->header().seqno(), pRobotMessage->header().tm(), pRobotMessage->header().mtype());
        _pose[0] =  pRobotMessage->feedback().cartesian().pos().x();
        _pose[1] =  pRobotMessage->feedback().cartesian().pos().y();
        _pose[2] =  pRobotMessage->feedback().cartesian().pos().z();

        _pose[3] =  pRobotMessage->feedback().cartesian().orient().u0();
        _pose[4] =  pRobotMessage->feedback().cartesian().orient().u1();
        _pose[5] =  pRobotMessage->feedback().cartesian().orient().u2();
        _pose[6] =  pRobotMessage->feedback().cartesian().orient().u3();
    }
    else
    {
        cout << "No header\n";
    }
}

void EGMClass::DisplayRobotMessage(EgmRobot *pRobotMessage)
{
    if (pRobotMessage->has_header() && pRobotMessage->header().has_seqno() && pRobotMessage->header().has_tm() && pRobotMessage->header().has_mtype())
    {
        printf("SeqNo=%d Tm=%u Type=%d\n", pRobotMessage->header().seqno(), pRobotMessage->header().tm(), pRobotMessage->header().mtype());
    }
    else
    {
        printf("No header\n");
    }
}
