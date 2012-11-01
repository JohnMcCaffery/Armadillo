/** 
 * @file llviewernui.h
 * @brief Viewer nui / NDOF device functionality.
 *
 * $LicenseInfo:firstyear=2001&license=viewerlgpl$
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

#ifndef LL_LLVIEWERNUI_H
#define LL_LLVIEWERNUI_H

#include "stdtypes.h"

#if LIB_NDOF
#include "ndofdev_external.h"
#else
#define NDOF_Device	void
#define NDOF_HotPlugResult S32
#endif

#include <NuiLib-API.h>


typedef enum e_nui_driver_state
{
	NUI_UNINITIALIZED,
	NUI_INITIALIZED,
	NUI_INITIALIZING
} ENuiDriverState;

class LLViewerNui : public LLSingleton<LLViewerNui>
{
public:
	LLViewerNui();
	virtual ~LLViewerNui();
	
	void init(bool autoenable);
	void terminate();

	void updateStatus();
	void scanNui();
	void moveObjects(bool reset = false);
	void moveAvatar(bool reset = false);
	void moveFlycam(bool reset = false);
	bool isNuiInitialized() const {return (mDriverState==NUI_INITIALIZED);}
	void setNeedsReset(bool reset = true) { mResetFlag = reset; }
	void setCameraNeedsUpdate(bool b)     { mCameraUpdated = b; }
	bool getCameraNeedsUpdate() const     { return mCameraUpdated; }
	bool getOverrideCamera() { return mOverrideCamera; }
	void setOverrideCamera(bool val);
	bool toggleFlycam();
	std::string getDescription();
	
protected:
	void updateEnabled(bool autoenable);
	void handleRun(F32 inc);
	void agentFly();
	void agentPitch(F32 pitch_inc);
	void agentYaw(F32 yaw_inc);
	void agentJump();
	
private:           
	//--Move--
//True if any of the movement conditions are met.
NuiLib::Condition				mCanMove;

//True if the user should be pushed forward.
NuiLib::Condition				mPush;

//True if yawing right or left.
NuiLib::Condition				mCanYaw;
//How far left or right to yaw (- = yaw left).
NuiLib::Scalar					mYaw;

//True if pitching up or down.
NuiLib::Condition				mCanPitch;
//How far up or down to pitch.
NuiLib::Scalar					mPitch;

//True if flying up or down
NuiLib::Condition				mCanFly;
//True if flying up, false if flying down.
NuiLib::Condition				mFly;

//--Manipulate
//True if translating using the right hand
NuiLib::Condition				mTranslateR;
//True if translating using the left hand
NuiLib::Condition				mTranslateL;
//True if rotating
NuiLib::Condition				mRotate;

//Pointing
//The X coordinate of the pointer
NuiLib::Scalar					mX;
//The Y coordinate of the pointer
NuiLib::Scalar					mY;

//Translate
//The translation delta for the right hand
NuiLib::Vector					mDeltaR;
//The translation delta for the left hand
NuiLib::Vector					mDeltaL;

//Rotate
//Rotation around the X axis
NuiLib::Scalar					mXRot;
//Rotation around the Y axis
NuiLib::Scalar					mYRot;
//Rotation around the Z axis
NuiLib::Scalar					mZRot;

NuiLib::Condition				mLClick;


ENuiDriverState	mDriverState;
NDOF_Device				*mNdofDev;
bool					mResetFlag;
F32						mPerfScale;
bool					mCameraUpdated;
bool 					mOverrideCamera;
U32						mNuiRun;
};

#endif
