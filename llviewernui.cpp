/** 
 * @file llviewernui.cpp
 * @brief Nui / NDOF device functionality.
 *
 * $LicenseInfo:firstyear=2002&license=viewerlgpl$
 * Second Life Viewer Source Code
 * Copyright (C) 2010, Linden Research, Inc.
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation;
 * version 2.1 of the License only.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 * 
 * Linden Research, Inc., 945 Battery Street, San Francisco, CA  94111  USA
 * $/LicenseInfo$
 */

#include "llviewerprecompiledheaders.h"

#include "llviewercontrol.h"
#include "llviewerwindow.h"
#include "llviewercamera.h"
#include "llappviewer.h"
#include "llkeyboard.h"
#include "lltoolmgr.h"
#include "llselectmgr.h"
#include "llviewermenu.h"
#include "llagent.h"
#include "llagentcamera.h"
#include "llfocusmgr.h"
#include "llwindow.h"

#include "llviewernui.h"

using namespace NuiLib;

// -----------------------------------------------------------------------------
void LLViewerNui::updateEnabled(bool autoenable)
{
	if (mDriverState == NUI_UNINITIALIZED)
	{
		gSavedSettings.setBOOL("NuiEnabled", FALSE );
	}
	if (!gSavedSettings.getBOOL("NuiEnabled"))
	{
		mOverrideCamera = FALSE;
	}
}

void LLViewerNui::setOverrideCamera(bool val)
{
	if (!gSavedSettings.getBOOL("NuiEnabled"))
	{
		mOverrideCamera = FALSE;
	}
	else
	{
		mOverrideCamera = val;
	}

	if (mOverrideCamera)
	{
		gAgentCamera.changeCameraToDefault();
	}
}

// -----------------------------------------------------------------------------
LLViewerNui::LLViewerNui()
:	mDriverState(NUI_UNINITIALIZED),
	mNdofDev(NULL),
	mResetFlag(false),
	mCameraUpdated(true),
	mOverrideCamera(false),
	mNuiRun(0)
{ }

// -----------------------------------------------------------------------------
LLViewerNui::~LLViewerNui()
{
	if (mDriverState == NUI_INITIALIZED)
	{
		terminate();
	}
}

#ifndef M_PI
#define M_PI 3.14159
#endif

const float R2DEG = (180 / (float) M_PI);

// -----------------------------------------------------------------------------
void LLViewerNui::init(bool autoenable)
{
	if (!  NuiFactory()->Init()) {
		mDriverState = NUI_UNINITIALIZED;
		return;
	}
	mDriverState = NUI_INITIALIZED;



	//Get the primary vectors.
	Vector shoulderR = joint(SHOULDER_RIGHT);
	Vector shoulderL = joint(SHOULDER_LEFT);
	Vector elbowR = joint(ELBOW_RIGHT);
	Vector elbowL = joint(ELBOW_LEFT);
	Vector wristR = joint(WRIST_RIGHT);
	Vector wristL = joint(WRIST_LEFT);
	Vector handR = joint(HAND_RIGHT);
	Vector handL = joint(HAND_LEFT);
	Vector hipC = joint(HIP_CENTER);
	Vector head = joint(HEAD);

	Vector yAxis = NuiLib::Vector("Y", 0.f, 1.f, 0.f);
	// Normal is the direction the camera is facing.
	Vector normal = Vector("Normal", 0, 0, 1);

	//Camera - If the right elbow is raised to be in line with the shoulders the camera is active.
	NuiLib::Vector upperArmCameraR = elbowR - shoulderR;
	NuiLib::Vector lowerArmCameraR = elbowR - wristR;
	Condition cameraActiveR = abs(x(upperArmCameraR)) > (abs(y(upperArmCameraR)) + abs(z(upperArmCameraR))) * 2.f;

	//Camera - If the right elbow is raised to be in line with the shoulders the camera is active.
	NuiLib::Vector upperArmCameraL = shoulderL - elbowL;
	NuiLib::Vector lowerArmCameraL = elbowL - wristL;
	Condition cameraActiveL = abs(x(upperArmCameraL)) > (abs(y(upperArmCameraL)) + abs(z(upperArmCameraL))) * 2.f;

	Condition cameraActive = cameraActiveL || cameraActiveR;
	//Condition cameraActive = cameraActiveR;
	
	// Normalize the distance between the shoulder and the right hand against the total length of the arm (armMax).
	// Once normalized constrain the value between the input from tracker PushD (starting at .8) and 1. 
	// So if the normalized value is .9 the constrained value is .5. Alternatively if the normalized value is .8 the constrained value is 0.
	//Scalar push = constrain(normalize(magnitude(armR), armMax), tracker("PushD", 25, .04f, 0.f, 20), 1.f, 0.f, false);
	// If the camera is inactive and the push value is > 0 and the forward component is significantly larger than the horizontal and vertical components combined.
	//mPush = !cameraActive && push > 0 && z(armR) > ((abs(x(armR) + abs(y(armR)))) * tracker("PushActive", 9, .5f, .5f, 5));

	//Pitch
	Scalar pitchArmD = tracker("PitchArmD", 20, 1.f, 0.f, 10);
	Scalar pitchArmR = tracker("PitchArmR", 40, 2.f, 10.f, 17);
	Scalar pitchArmG = tracker("PitchArmG", 30, 1.f, 0.f, 15);
	Scalar pitchAS = tracker("PitchAS", 29, 1.f, 1.f, 20);

	Vector vPlaneCameraR = NuiLib::limit(lowerArmCameraR, false, true, true);
	Vector vPlaneCameraL = NuiLib::limit(lowerArmCameraL, false, true, true);
	// Pitch is the angle between normal and the vertical component of the vector between right shoulder and right hand.
	Scalar pitchR = NuiLib::acos(dot(normalize(vPlaneCameraR), normal)) * invert(x(cross(normal, vPlaneCameraR)) >= 0);
	Scalar pitchL = NuiLib::acos(dot(normalize(vPlaneCameraL), normal)) * invert(x(cross(normal, vPlaneCameraL)) >= 0);
	// Constrain the pitch value by 3 values input by 3 trackers.
	pitchR = constrain(pitchR * R2DEG, pitchArmD, pitchArmR, pitchArmG, true) / pitchAS;
	pitchL = constrain(pitchL * R2DEG, pitchArmD, pitchArmR, pitchArmG, true) / pitchAS;
	mPitch = ifScalar(cameraActiveR, pitchR, .0f) + ifScalar(cameraActiveL, pitchL, .0f);

	mCanPitch = cameraActive && mPitch != 0.f;

	//Yaw - Yaw has 3 components. The camera arm. The horizontal lean (head vs hip centre) and the twist of the shoulders.
	Scalar yawArmD = tracker("YawArmD", 20, 1.f, 0.f, 10);
	Scalar yawArmR = tracker("YawArmR", 40, 2.f, 10.f, 15);
	Scalar yawArmG = tracker("YawArmG", 20, 1.f, 0.f, 10);
	Scalar yawLeanD = tracker("YawLeanD", 20, .5f, 0.f, 10);
	Scalar yawLeanR = tracker("YawLeanR", 20, 1.f, 0.f, 15);
	Scalar yawLeanG = tracker("YawLeanG", 50, 1.f, 0.f, 30);
	Scalar yawTwistD = tracker("YawTwistD", 10, .025f, 0.f, 6);
	Scalar yawTwistR = tracker("YawTwistR", 20, .05f, 0.f, 9);
	Scalar yawTwistG = tracker("YawTwistG", 20, 1.f, 0.f, 10);
	
	Vector hPlaneCameraR = NuiLib::limit(lowerArmCameraR, true, false, true);
	Vector hPlaneCameraL = NuiLib::limit(lowerArmCameraL, true, false, true);
	// Yaw component 1 is the angle between normal and the horizontal component of the vector between right shoulder and right hand.
	Scalar yawCameraR = NuiLib::acos(dot(normalize(hPlaneCameraR), normal)) * invert(y(cross(normal, hPlaneCameraR)) >= 0);
	Scalar yawCameraL = NuiLib::acos(dot(normalize(hPlaneCameraL), normal)) * invert(y(cross(normal, hPlaneCameraL)) >= 0);
	// Constrain the component value by 3 values input by 3 trackers.
	yawCameraR = constrain(yawCameraR * R2DEG, yawArmD, yawArmR, yawArmG, true) / tracker("YawAS", 29, 1.f, 1.f, 20);
	yawCameraL = constrain(yawCameraL * R2DEG, yawArmD, yawArmR, yawArmG, true) / tracker("YawAS", 29, 1.f, 1.f, 20);
	//Only take the value if camera is active
	yawCameraR = ifScalar(cameraActiveR, yawCameraR, .0f);
	yawCameraL = ifScalar(cameraActiveL, yawCameraL, .0f);

	Vector yawCore = limit(head - hipC, true, true, false);
	// Yaw component 2 is how far the user is leaning horizontally. This is calculated the angle between vertical and the vector between the hip centre and the head.
	Scalar yawLean = NuiLib::acos(dot(normalize(yawCore), yAxis)) * invert(z(cross(yawCore, yAxis)) >= 0);
	// Constrain the component value by 3 values input by 3 trackers.
	yawLean = constrain(yawLean * R2DEG, yawLeanD, yawLeanR, yawLeanG, true) / tracker("YawLS", 29, 1.f, 1.f, 20);
	
	Vector shoulderDiff = shoulderR - shoulderL;
	// Yaw component 3 is the twist of the shoulders. This is calculated as the difference between the two z values.
	Scalar yawTwist = z(shoulderDiff) / magnitude(shoulderDiff);
	// Constrain the component value by 3 values input by 3 trackers.
	yawTwist = constrain(yawTwist, yawTwistD, yawTwistR, yawTwistG, true) / tracker("YawTS", 29, 1.f, 1.f, 20);

	// Combine all 3 components into the final yaw value.
	mYaw = yawCameraR + yawCameraL + yawLean + yawTwist;
	mCanYaw = (cameraActive && (yawCameraR + yawCameraL) != 0.f) || yawLean != 0.f || yawTwist != 0.f;

	Scalar flyUpD = tracker("FlyUpD", 120, 1.f, 0.f, 65);
	Scalar flyUpR = tracker("FlyUpR", 120, 1.f, 0.f, 50);
	Scalar flyDownD = tracker("FlyDownD", 120, 1.f, 0.f, 45);
	Scalar flyDownR = tracker("FlyDownR", 120, 1.f, 0.f, 15);

	//Fly
	NuiLib::Vector armR = shoulderR - handR;
	NuiLib::Vector vPlaneR = limit(armR, false, true, true);
	// The angle between normal and the vector between the shoulder and the hand.
	Scalar flyR = NuiLib::acos(NuiLib::dot(normalize(vPlaneR), normal));
	// Constrain the positive angle to go up past vertical.
	Scalar upR = constrain(flyR * R2DEG, flyUpD, flyUpR, 0.f, true); //Constraints if R is raised
	// Constrain the negative angle to stop before vertical so that hands lying by the side doesn't trigger flying down.
	Scalar downR = constrain(flyR * R2DEG, flyDownD, flyDownR, 0.f, true); //Constraints if R is lowered
	// Whether the arm is raised or lowered.
	Condition dirR = NuiLib::x(cross(normal, vPlaneR)) >= 0; 
	// Whether R is in range to fly
	Condition flyCondR = (magnitude(vPlaneR) > 0.f) && ((dirR && (upR > 0)) || ((!dirR) && (downR > 0))); 

	NuiLib::Vector armL = shoulderL - handL;
	NuiLib::Vector vPlaneL = limit(armL, false, true, true);
	// The angle between normal and the vector between the shoulder and the hand.
	Scalar flyL = NuiLib::acos(NuiLib::dot(normalize(vPlaneL), normal)) * R2DEG;
	// Constrain the positive angle to go up past vertical.
	Scalar upL = constrain(flyL, flyUpD, flyUpR, 0.f, true); //Constraints if L is raised
	// Constrain the negative angle to stop before vertical so that hands lying by the side doesn't trigger flying down.
	Scalar downL = constrain(flyL, flyDownD, flyDownR, 0.f, true); //Constraints if L is lowered
	// Whether the arm is raised or lowered.
	Condition dirL = NuiLib::x(cross(normal, vPlaneL)) >= 0; 
	// Whether L is in range to fly
	Condition flyCondL = magnitude(vPlaneL) > 0.f && (dirL && upL > 0.f) || (!dirL && downL > 0.f); 

	//Up trumps down
	mFly = (dirR && flyCondR) || (dirL && flyCondL);
	// Fly if camera is inactive and flying with right or left arm
	mCanFly = (flyCondR && !cameraActiveR) || (flyCondL && !cameraActiveL);

	Scalar pushThresh = tracker("PushThreshold", 30, .05f, .0f, 9);
	Condition pushR = z(shoulderR) - z(handR) > pushThresh; 
	Condition pushL = z(shoulderL) - z(handL) > pushThresh; 
	mPush = (pushR && !cameraActiveR) || (pushL && !cameraActiveL);

	mCanMove = mPush || mCanYaw || mCanPitch || mCanFly;



	/*mLClick = fist(true);
	mLClick.OnTrue([](IObservable *s) {
		//LLMouseHandler* mouse_captor = gFocusMgr.getMouseCapture();
		//if( mouse_captor ) {
			//S32 x, y;
			//LLUI::getMousePositionScreen(&x, &y);
			//S32 local_x, local_y;
			//mouse_captor->screenPointToLocal( x, y, &local_x, &local_y );
			////if (LLView::sDebugMouseHandling) {
				//llinfos << "left" << " Kinect " << "click" << " handled by captor " << mouse_captor->getName() << llendl;
			//}
			//return mouse_captor->handleAnyMouseClick(local_x, local_y, gKeyboard->currentMask(TRUE), LLMouseHandler::CLICK_LEFT, false);
		//}
	});*/

	NuiFactory()->SetAutoPoll(true);
}

void LLViewerNui::scanNui()
{
	if (mDriverState != NUI_INITIALIZED/* || !gSavedSettings.getBOOL("NuiEnabled")*/)
	{
		return;
	}

	// App focus check Needs to happen AFTER updateStatus in case the nui
	// is not centred when the app loses focus.
	if (!gFocusMgr.getAppHasFocus()) {
		return;
	}


	if (*mLClick) {
		S32 x, y;
		LLUI::getMousePositionScreen(&x, &y);
		LLUI::setMousePositionScreen(x++, y++);
		cout << "X: " << x << " - Y: " << y << '\n';
	}

	if (*mCanMove/* && LLSelectMgr::getInstance()->getSelection().isNull()*/) {
		if (*mPush)
			gAgent.moveAt(1, false);
		if (*mCanYaw)
			agentYaw(*mYaw);
		//if (*mCanPitch)
			agentPitch(*mPitch);
		if (*mCanFly)
			agentFly();
	} else {
		agentYaw(0.f);
		agentPitch(0.f);
		gAgent.moveAt(0, false);
		gAgent.moveUp(0.f);
	}
}

// -----------------------------------------------------------------------------
void LLViewerNui::moveObjects(bool reset)
{
	static bool toggle_send_to_sim = false;

	if (!gFocusMgr.getAppHasFocus() || mDriverState != NUI_INITIALIZED) {
		return;
	}



	// Clear AFK state if moved beyond the deadzone
	if (gAwayTimer.getElapsedTimeF32() > LLAgent::MIN_AFK_TIME)
		gAgent.clearAFK();

	LLVector3 v;
	if (*mRotate) {
		if (LLSelectMgr::getInstance()->selectionMove(v, *mXRot, *mYRot, *mZRot, UPD_ROTATION)) 
			toggle_send_to_sim = true;
	} /*else if (**mTranslateL) {
		v.setVec(mLeftDelta->GetX(), mLeftDelta->GetY(), mLeftDelta->GetZ());
		if (LLSelectMgr::getInstance()->selectionMove(v, 0, 0, 0, UPD_POSITION)) 
			toggle_send_to_sim = true;
	} else if (**mTranslateR) {
		v.setVec(mRightDelta->GetX(), mRightDelta->GetY(), mRightDelta->GetZ());
		if (LLSelectMgr::getInstance()->selectionMove(v, 0, 0, 0, UPD_POSITION)) 
			toggle_send_to_sim = true;
	}*/

	// the selection update could fail, so we won't send 
	if (toggle_send_to_sim)
	{
		LLSelectMgr::getInstance()->sendSelectionMove();
		toggle_send_to_sim = false;
	}
}

// -----------------------------------------------------------------------------
void LLViewerNui::agentFly()
{
	if (*mFly && (!(gAgent.getFlying() ||
		!gAgent.canFly() ||
		gAgent.upGrabbed() ||
		!gSavedSettings.getBOOL("AutomaticFly"))) )
	{
		gAgent.setFlying(true);
	}
	gAgent.moveUp(*mFly ? 1 : -1);
}

// -----------------------------------------------------------------------------
void LLViewerNui::agentPitch(F32 pitch_inc)
{
	if (pitch_inc < 0)
	{
		gAgent.setControlFlags(AGENT_CONTROL_PITCH_POS);
	}
	else if (pitch_inc > 0)
	{
		gAgent.setControlFlags(AGENT_CONTROL_PITCH_NEG);
	}
	
	gAgent.pitch(-pitch_inc);
}

// -----------------------------------------------------------------------------
void LLViewerNui::agentYaw(F32 yaw_inc)
{	
	// Cannot steer some vehicles in mouselook if the script grabs the controls
	if (gAgentCamera.cameraMouselook() && !gSavedSettings.getBOOL("NuiMouselookYaw"))
	{
		gAgent.rotate(-yaw_inc, gAgent.getReferenceUpVector());
	}
	else
	{
		if (yaw_inc < 0)
		{
			gAgent.setControlFlags(AGENT_CONTROL_YAW_POS);
		}
		else if (yaw_inc > 0)
		{
			gAgent.setControlFlags(AGENT_CONTROL_YAW_NEG);
		}

		gAgent.yaw(-yaw_inc);
	}
}

// -----------------------------------------------------------------------------
void LLViewerNui::terminate()
{
#if LIB_NDOF

	ndof_libcleanup();
	llinfos << "Terminated connection with NDOF device." << llendl;
	mDriverState = NUI_UNINITIALIZED;
#endif
}


// -----------------------------------------------------------------------------
void LLViewerNui::handleRun(F32 inc)
{
	// Decide whether to walk or run by applying a threshold, with slight
	// hysteresis to avoid oscillating between the two with input spikes.
	// Analog speed control would be better, but not likely any time soon.
	if (inc > gSavedSettings.getF32("NuiRunThreshold"))
	{
		if (1 == mNuiRun)
		{
			++mNuiRun;
			gAgent.setRunning();
			gAgent.sendWalkRun(gAgent.getRunning());
		}
		else if (0 == mNuiRun)
		{
			// hysteresis - respond NEXT frame
			++mNuiRun;
		}
	}
	else
	{
		if (mNuiRun > 0)
		{
			--mNuiRun;
			if (0 == mNuiRun)
			{
				gAgent.clearRunning();
				gAgent.sendWalkRun(gAgent.getRunning());
			}
		}
	}
}



// -----------------------------------------------------------------------------
void LLViewerNui::moveAvatar(bool reset)
{
	
}

// -----------------------------------------------------------------------------
void LLViewerNui::moveFlycam(bool reset)
{
	/*
	sFlycamPosition += LLVector3(sDelta) * sFlycamRotation;

	LLMatrix3 rot_mat(sDelta[3], sDelta[4], sDelta[5]);
	sFlycamRotation = LLQuaternion(rot_mat)*sFlycamRotation;

	if (gSavedSettings.getBOOL("AutoLeveling"))
	{
		LLMatrix3 level(sFlycamRotation);

		LLVector3 x = LLVector3(level.mMatrix[0]);
		LLVector3 y = LLVector3(level.mMatrix[1]);
		LLVector3 z = LLVector3(level.mMatrix[2]);

		y.mV[2] = 0.f;
		y.normVec();

		level.setRows(x,y,z);
		level.orthogonalize();
				
		LLQuaternion quat(level);
		sFlycamRotation = nlerp(llmin(feather*time,1.f), sFlycamRotation, quat);
	}

	if (gSavedSettings.getBOOL("ZoomDirect"))
	{
		sFlycamZoom = sLastDelta[6]*axis_scale[6]+dead_zone[6];
	}
	else
	{
		sFlycamZoom += sDelta[6];
	}

	LLMatrix3 mat(sFlycamRotation);

	LLViewerCamera::getInstance()->setView(sFlycamZoom);
	LLViewerCamera::getInstance()->setOrigin(sFlycamPosition);
	LLViewerCamera::getInstance()->mXAxis = LLVector3(mat.mMatrix[0]);
	LLViewerCamera::getInstance()->mYAxis = LLVector3(mat.mMatrix[1]);
	LLViewerCamera::getInstance()->mZAxis = LLVector3(mat.mMatrix[2]);
	*/
}

// -----------------------------------------------------------------------------
bool LLViewerNui::toggleFlycam()
{
	if (!gSavedSettings.getBOOL("NuiEnabled") || !gSavedSettings.getBOOL("NuiFlycamEnabled"))
	{
		mOverrideCamera = false;
		return false;
	}

	if (!mOverrideCamera)
	{
		gAgentCamera.changeCameraToDefault();
	}

	if (gAwayTimer.getElapsedTimeF32() > LLAgent::MIN_AFK_TIME)
	{
		gAgent.clearAFK();
	}
	
	mOverrideCamera = !mOverrideCamera;
	if (mOverrideCamera)
	{
		moveFlycam(true);
		
	}
	else 
	{
		// Exiting from the flycam mode: since we are going to keep the flycam POV for
		// the main camera until the avatar moves, we need to track this situation.
		setCameraNeedsUpdate(false);
		setNeedsReset(true);
	}
	return true;
}
// -----------------------------------------------------------------------------
std::string LLViewerNui::getDescription()
{
	std::string res;
#if LIB_NDOF
	if (mDriverState == NUI_INITIALIZED && mNdofDev)
	{
		res = ll_safe_string(mNdofDev->product);
	}
#endif
	return res;
}
