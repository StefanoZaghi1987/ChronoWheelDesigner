// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================
//
// FEA for 3D beams
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono/solver/ChSolverMINRES.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/collision/ChCCollisionSystemBullet.h"

#include "chrono_fea/ChElementBeamEuler.h"
#include "chrono_fea/ChBuilderBeam.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_fea/ChLinkDirFrame.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono_fea/ChContactSurfaceMesh.h"
#include "chrono_fea/ChContactSurfaceNodeCloud.h"

#include "chrono_mkl/ChSolverMKL.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;

using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;


// A helper function that creates a'lobed gear', almost a macro user in main() 
// to quickly create one or two rotating obstacles for the extruding beam 

std::shared_ptr<ChBody> CreateLobedGear(
	ChVector<> gear_center,
	int    lobe_copies,
	double lobe_width,
	double lobe_primitive_rad,
	double lobe_inner_rad,
	double lobe_outer_rad,
	double lobe_thickness,
	ChSystem& my_system,
	std::shared_ptr<ChMaterialSurface> mysurfmaterial
) {

	auto mgear = std::make_shared<ChBody>();
	mgear->SetMaterialSurface(mysurfmaterial);
	mgear->SetPos(gear_center);
	my_system.Add(mgear);

	// add cylindrical lobes
	mgear->GetCollisionModel()->ClearModel();
	for (int i = 0; i< lobe_copies; ++i) {
		double phase = CH_C_2PI * ((double)i / (double)lobe_copies);
		// this is a quick shortcut from ChUtilsCreators.h, 
		// it both adds the collision shape and the visualization asset:
		/*
		utils::AddCylinderGeometry(
		mgear.get(),
		lobe_width*0.5,
		lobe_thickness*0.5,
		ChVector<>(lobe_primitive_rad*sin(phase), lobe_primitive_rad*cos(phase),0),
		Q_from_AngAxis(CH_C_PI_2, VECT_X), // rotate cylinder axis: from default on Y axis, to Z axis
		true);
		*/
		utils::AddBoxGeometry(
			mgear.get(),
			ChVector<>(lobe_width, lobe_outer_rad - lobe_inner_rad, lobe_thickness)*0.5, // half size used in this function 
			ChVector<>(0.5*(lobe_outer_rad + lobe_inner_rad)*sin(phase), 0.5*(lobe_outer_rad + lobe_inner_rad)*cos(phase), 0),
			Q_from_AngAxis(-phase, VECT_Z), // rotate cylinder axis: from default on Y axis, to Z axis
			true);
	}
	utils::AddCylinderGeometry(mgear.get(), lobe_inner_rad, lobe_thickness*0.5, ChVector<>(0, 0, 0), Q_from_AngAxis(CH_C_PI_2, VECT_X), true);
	mgear->GetCollisionModel()->BuildModel();
	mgear->SetCollide(true);

	return mgear;
}



// Define an enum class to identity GUI controls
enum
{
	GUI_ID_SCROLLBAR_WireElementLength = 100,
	GUI_ID_SCROLLBAR_WireDiameter = 101,
	GUI_ID_SCROLLBAR_WireYoungModulus = 102,
	GUI_ID_SCROLLBAR_WireShearModulus = 103,
	GUI_ID_SCROLLBAR_WireRayleighDamping = 104,
	GUI_ID_SPINBOX_BrakeGearLobeNumber = 105,
	GUI_ID_SCROLLBAR_BrakeGearLobeWidth = 106,
	GUI_ID_SCROLLBAR_BrakeGearLobeThickness = 107,
	GUI_ID_SCROLLBAR_BrakeGearLobePrimitiveRadius = 108,
	GUI_ID_SCROLLBAR_BrakeGearLobeInnerRadius = 109,
	GUI_ID_SCROLLBAR_BrakeGearLobeOuterRadius = 110,
	GUI_ID_SCROLLBAR_ExtruderSpeed = 111,
	GUI_ID_SCROLLBAR_BrakeGearSpeed = 112,
	GUI_ID_SCROLLBAR_ExtrusionBrakeSpeedRatio = 113,
	GUI_ID_BUTTON_ConfigurationCompleted = 114,
	GUI_ID_BUTTON_StartSimulation = 115,
	GUI_ID_BUTTON_PauseSimulation = 116,
	GUI_ID_BUTTON_StopSimulation = 117
};

// Define flags for simulation management
bool isConfigurationCompleted = false;
bool isSimulationStarted = false;
bool isSimulationPaused = false;


// Define a MyEventReceiver class which will be used to manage input
// from the Irrlicht GUI graphical user interface

class MyEventReceiver : public IEventReceiver {
public:
	MyEventReceiver(ChSystemSMC* asystem, ChIrrApp* application, IrrlichtDevice* adevice,
		std::shared_ptr<chrono::ChBody> aground,
		std::shared_ptr<chrono::ChMaterialSurfaceSMC> asurfacematerial,
		std::shared_ptr<chrono::fea::ChBeamSectionAdvanced> awire,
		std::shared_ptr<chrono::fea::ChExtruderBeamEuler> aWireExtruder,
		std::shared_ptr<chrono::ChBody> aLowBrakeGear,
		std::shared_ptr<chrono::ChLinkMotorRotationSpeed> aLowBrakeGearLink,
		std::shared_ptr<chrono::ChBody> aHighBrakeGear,
		std::shared_ptr<chrono::ChLinkMotorRotationSpeed> aHighBrakeGearLink
	)
	{
		// store pointer to physical system & other stuff so we can tweak them by user keyboard
		msystem = asystem;
		mapplication = application;
		mdevice = adevice;

		mground = aground;
		msurfacematerial = asurfacematerial;
		mwire = awire;
		mWireExtruder = aWireExtruder;
		mLowBrakeGear = aLowBrakeGear;
		mLowBrakeGearLink = aLowBrakeGearLink;
		mHighBrakeGear = aHighBrakeGear;
		mHighBrakeGearLink = aHighBrakeGearLink;

		mdevice->setEventReceiver(this);

		char message[50];

		// ..add a GUI text and GUI slider to control the wire element length (discretization size, in m)
		scrollbar_WireElementLength = mdevice->getGUIEnvironment()->addScrollBar(true, rect<s32>(10, 85, 150, 100), 0, GUI_ID_SCROLLBAR_WireElementLength);
		scrollbar_WireElementLength->setMin(10);
		scrollbar_WireElementLength->setMax(500);
		scrollbar_WireElementLength->setPos((s32)(50 + 50.0 * (aWireExtruder->GetBeamElementLength() - 0.005) / 0.005));
		text_WireElementLength = mdevice->getGUIEnvironment()->addStaticText(L"Wire element length [m]:", rect<s32>(150, 85, 400, 100), false);

		// show wire element length (discretization size, in m) as formatted text in interface screen
		double wireElementLength = 0.005 + 0.005 * (((double)(scrollbar_WireElementLength->getPos() - 50)) / 50.0);
		sprintf(message, "Wire element length [m]: %g", wireElementLength);
		text_WireElementLength->setText(core::stringw(message).c_str());

		// ..add a GUI text and GUI slider to control the wire diameter (m)
		scrollbar_WireDiameter = mdevice->getGUIEnvironment()->addScrollBar(true, rect<s32>(10, 110, 150, 125), 0, GUI_ID_SCROLLBAR_WireDiameter);
		scrollbar_WireDiameter->setMin(10);
		scrollbar_WireDiameter->setMax(300);
		scrollbar_WireDiameter->setPos((s32)(50 + 50.0 * ((awire->GetDrawCircularRadius() * 2) - 0.005) / 0.005));
		text_WireDiameter = mdevice->getGUIEnvironment()->addStaticText(L"Wire diameter [m]:", rect<s32>(150, 110, 400, 125), false);

		// show wire diameter (m) as formatted text in interface screen
		double wireDiameter = 0.005 + 0.005 * (((double)(scrollbar_WireDiameter->getPos() - 50)) / 50.0);
		sprintf(message, "Wire diameter [m]: %g", wireDiameter);
		text_WireDiameter->setText(core::stringw(message).c_str());

		// ..add a GUI text and GUI slider to control the wire Young elastic modulus (N/m^2)
		scrollbar_WireYoungModulus = mdevice->getGUIEnvironment()->addScrollBar(true, rect<s32>(10, 135, 150, 150), 0, GUI_ID_SCROLLBAR_WireYoungModulus);
		scrollbar_WireYoungModulus->setMin(10);
		scrollbar_WireYoungModulus->setMax(250000);
		scrollbar_WireYoungModulus->setPos((s32)(50 + 50.0 * (awire->GetYoungModulus() - 0.05e9) / 0.05e9));
		text_WireYoungModulus = mdevice->getGUIEnvironment()->addStaticText(L"Wire Young elastic modulus [N/m^2]:", rect<s32>(150, 135, 400, 150), false);
		
		// show wire Young modulus as formatted text in interface screen
		double wireYoungModulus = 0.05e9 + 0.05e9 * (((double)(scrollbar_WireYoungModulus->getPos() - 50)) / 50.0);
		sprintf(message, "Wire Young elastic modulus [N/m^2]: %g", wireYoungModulus);
		text_WireYoungModulus->setText(core::stringw(message).c_str());

		// ..add a GUI text and GUI slider to control the wire shear modulus (N/m^2)
		scrollbar_WireShearModulus = mdevice->getGUIEnvironment()->addScrollBar(true, rect<s32>(10, 160, 150, 175), 0, GUI_ID_SCROLLBAR_WireShearModulus);
		scrollbar_WireShearModulus->setMin(10);
		scrollbar_WireShearModulus->setMax(2500000);
		scrollbar_WireShearModulus->setPos((s32)(50 + 50.0 * (awire->GetGshearModulus() - 0.005e9) / 0.005e9));
		text_WireShearModulus = mdevice->getGUIEnvironment()->addStaticText(L"Wire shear modulus [N/m^2]:", rect<s32>(150, 160, 400, 175), false);

		// show wire shear modulus as formatted text in interface screen
		double wireShearModulus = 0.005e9 + 0.005e9 * (((double)(scrollbar_WireShearModulus->getPos() - 50)) / 50.0);
		sprintf(message, "Wire shear modulus [N/m^2]: %g", wireShearModulus);
		text_WireShearModulus->setText(core::stringw(message).c_str());

		// ..add a GUI text and GUI slider to control the wire Rayleigh damping ratio
		scrollbar_WireRayleighDamping = mdevice->getGUIEnvironment()->addScrollBar(true, rect<s32>(10, 185, 150, 200), 0, GUI_ID_SCROLLBAR_WireRayleighDamping);
		scrollbar_WireRayleighDamping->setMin(10);
		scrollbar_WireRayleighDamping->setMax(1000);
		scrollbar_WireRayleighDamping->setPos((s32)(50 + 50.0 * (awire->GetBeamRaleyghDamping() - 0.05) / 0.05));
		text_WireRayleighDamping = mdevice->getGUIEnvironment()->addStaticText(L"Wire Rayleigh damping ratio:", rect<s32>(150, 185, 400, 200), false);

		// show wire Rayleigh damping ratio as formatted text in interface screen
		double wireRayleighDamping = 0.05 + 0.05 * (((double)(scrollbar_WireRayleighDamping->getPos() - 50)) / 50.0);
		sprintf(message, "Wire Rayleigh damping ratio: %g", wireRayleighDamping);
		text_WireRayleighDamping->setText(core::stringw(message).c_str());

		// ..add a GUI text and GUI slider to control the wire Young elastic modulus (N/m^2)
		spinbox_BrakeGearLobeNumber = mdevice->getGUIEnvironment()->addSpinBox(L"8", rect<s32>(10, 210, 150, 225), true, 0, GUI_ID_SPINBOX_BrakeGearLobeNumber);
		spinbox_BrakeGearLobeNumber->setValue(8);
		text_BrakeGearLobeNumber = mdevice->getGUIEnvironment()->addStaticText(L"Brake gear lobe number:", rect<s32>(150, 210, 400, 225), false);

		// show wire Young modulus as formatted text in interface screen
		s32 brakeGearLobeNumber = spinbox_BrakeGearLobeNumber->getValue();
		sprintf(message, "Brake gear lobe number: %d", brakeGearLobeNumber);
		text_BrakeGearLobeNumber->setText(core::stringw(message).c_str());

		// ..add a GUI text and GUI slider to control the break gear lobe width (m)
		scrollbar_BrakeGearLobeWidth = mdevice->getGUIEnvironment()->addScrollBar(true, rect<s32>(10, 235, 150, 250), 0, GUI_ID_SCROLLBAR_BrakeGearLobeWidth);
		scrollbar_BrakeGearLobeWidth->setMin(10);
		scrollbar_BrakeGearLobeWidth->setMax(500);
		scrollbar_BrakeGearLobeWidth->setPos((s32)(50 + 50.0 * (0.03 - 0.005) / 0.005));
		text_BrakeGearLobeWidth = mdevice->getGUIEnvironment()->addStaticText(L"Brake gear lobe width [m]:", rect<s32>(150, 235, 400, 250), false);

		// show brake gear lobe width as formatted text in interface screen
		double brakeGearLobeWidth = 0.005 + 0.005 * (((double)(scrollbar_BrakeGearLobeWidth->getPos() - 50)) / 50.0);
		sprintf(message, "Brake gear lobe width [m]: %g", brakeGearLobeWidth);
		text_BrakeGearLobeWidth->setText(core::stringw(message).c_str());

		// ..add a GUI text and GUI slider to control the break gear lobe thickness (m)
		scrollbar_BrakeGearLobeThickness = mdevice->getGUIEnvironment()->addScrollBar(true, rect<s32>(10, 260, 150, 275), 0, GUI_ID_SCROLLBAR_BrakeGearLobeThickness);
		scrollbar_BrakeGearLobeThickness->setMin(10);
		scrollbar_BrakeGearLobeThickness->setMax(2000);
		scrollbar_BrakeGearLobeThickness->setPos((s32)(50 + 50.0 * (0.08 - 0.005) / 0.005));
		text_BrakeGearLobeThickness = mdevice->getGUIEnvironment()->addStaticText(L"Brake gear lobe thickness [m]:", rect<s32>(150, 260, 400, 275), false);

		// show brake gear lobe thickness as formatted text in interface screen
		double brakeGearLobeThickness = 0.005 + 0.005 * (((double)(scrollbar_BrakeGearLobeThickness->getPos() - 50)) / 50.0);
		sprintf(message, "Brake gear lobe thickness [m]: %g", brakeGearLobeThickness);
		text_BrakeGearLobeThickness->setText(core::stringw(message).c_str());

		// ..add a GUI text and GUI slider to control the break gear lobe primitive radius (m)
		scrollbar_BrakeGearLobePrimitiveRadius = mdevice->getGUIEnvironment()->addScrollBar(true, rect<s32>(10, 285, 150, 300), 0, GUI_ID_SCROLLBAR_BrakeGearLobePrimitiveRadius);
		scrollbar_BrakeGearLobePrimitiveRadius->setMin(10);
		scrollbar_BrakeGearLobePrimitiveRadius->setMax(5000);
		scrollbar_BrakeGearLobePrimitiveRadius->setPos((s32)(50 + 50.0 * (0.3 - 0.005) / 0.005));
		text_BrakeGearLobePrimitiveRadius = mdevice->getGUIEnvironment()->addStaticText(L"Brake gear lobe primitive radius [m]:", rect<s32>(150, 285, 400, 300), false);

		// show brake gear lobe primitive radius as formatted text in interface screen
		double brakeGearLobePrimitiveRadius = 0.005 + 0.005 * (((double)(scrollbar_BrakeGearLobePrimitiveRadius->getPos() - 50)) / 50.0);
		sprintf(message, "Brake gear lobe primitive radius [m]: %g", brakeGearLobePrimitiveRadius);
		text_BrakeGearLobePrimitiveRadius->setText(core::stringw(message).c_str());

		// ..add a GUI text and GUI slider to control the break gear lobe inner radius (m)
		scrollbar_BrakeGearLobeInnerRadius = mdevice->getGUIEnvironment()->addScrollBar(true, rect<s32>(10, 310, 150, 325), 0, GUI_ID_SCROLLBAR_BrakeGearLobeInnerRadius);
		scrollbar_BrakeGearLobeInnerRadius->setMin(10);
		scrollbar_BrakeGearLobeInnerRadius->setMax(5000);
		scrollbar_BrakeGearLobeInnerRadius->setPos((s32)(50 + 50.0 * (0.23 - 0.005) / 0.005));
		text_BrakeGearLobeInnerRadius = mdevice->getGUIEnvironment()->addStaticText(L"Brake gear lobe inner radius [m]:", rect<s32>(150, 310, 400, 325), false);

		// show brake gear lobe inner radius as formatted text in interface screen
		double brakeGearLobeInnerRadius = 0.005 + 0.005 * (((double)(scrollbar_BrakeGearLobeInnerRadius->getPos() - 50)) / 50.0);
		sprintf(message, "Brake gear lobe inner radius [m]: %g", brakeGearLobeInnerRadius);
		text_BrakeGearLobeInnerRadius->setText(core::stringw(message).c_str());

		// ..add a GUI text and GUI slider to control the break gear lobe outer radius (m)
		scrollbar_BrakeGearLobeOuterRadius = mdevice->getGUIEnvironment()->addScrollBar(true, rect<s32>(10, 335, 150, 350), 0, GUI_ID_SCROLLBAR_BrakeGearLobeOuterRadius);
		scrollbar_BrakeGearLobeOuterRadius->setMin(10);
		scrollbar_BrakeGearLobeOuterRadius->setMax(5000);
		scrollbar_BrakeGearLobeOuterRadius->setPos((s32)(50 + 50.0 * (0.34 - 0.005) / 0.005));
		text_BrakeGearLobeOuterRadius = mdevice->getGUIEnvironment()->addStaticText(L"Brake gear lobe outer radius [m]:", rect<s32>(150, 335, 400, 350), false);

		// show brake gear lobe outer radius as formatted text in interface screen
		double brakeGearLobeOuterRadius = 0.005 + 0.005 * (((double)(scrollbar_BrakeGearLobeOuterRadius->getPos() - 50)) / 50.0);
		sprintf(message, "Brake gear lobe outer radius [m]: %g", brakeGearLobeOuterRadius);
		text_BrakeGearLobeOuterRadius->setText(core::stringw(message).c_str());

		// ..add a GUI text and GUI slider to control the wire extruder speed [rad/s]
		scrollbar_ExtruderSpeed = mdevice->getGUIEnvironment()->addScrollBar(true, rect<s32>(10, 360, 150, 375), 0, GUI_ID_SCROLLBAR_ExtruderSpeed);
		scrollbar_ExtruderSpeed->setMax(8000);
		scrollbar_ExtruderSpeed->setPos((s32)(50 + 50.0 * ((abs(aWireExtruder->GetExtruderSpeed()) - 0.05) / 0.05)));
		text_ExtruderSpeed = mdevice->getGUIEnvironment()->addStaticText(L"Wire extruder speed [rad/s]:", rect<s32>(150, 360, 400, 375), false);

		// show extruder speed as formatted text in interface screen
		double extruderSpeed = 0.05 + 0.05 * ((double)(scrollbar_ExtruderSpeed->getPos() - 50) / 50.0);
		sprintf(message, "Wire extruder speed [rad/s]: %g", extruderSpeed);
		text_ExtruderSpeed->setText(core::stringw(message).c_str());

		// ..add a GUI text and GUI slider to control the brake gear speed [rad/s]
		scrollbar_BrakeGearSpeed = mdevice->getGUIEnvironment()->addScrollBar(true, rect<s32>(10, 385, 150, 400), 0, GUI_ID_SCROLLBAR_BrakeGearSpeed);
		scrollbar_BrakeGearSpeed->setMax(2000);
		/*scrollbar_BrakeGearSpeed->setPos((s32)(50 + 50.0 * ((abs(aHighBrakeGearLink->GetSpeedFunction()->Get_y(0)) - 0.1) / 0.1)));*/
		scrollbar_BrakeGearSpeed->setPos(50);
		text_BrakeGearSpeed = mdevice->getGUIEnvironment()->addStaticText(L"Brake gear speed [rad/s]:", rect<s32>(150, 385, 400, 400), false);

		// show brake gear speed as formatted text in interface screen
		double brakeGearSpeed = 0.1 + 0.1 * ((double)(scrollbar_BrakeGearSpeed->getPos() - 50) / 50.0);
		sprintf(message, "Brake gear speed [rad/s]: %g", brakeGearSpeed);
		text_BrakeGearSpeed->setText(core::stringw(message).c_str());

		// ..add a GUI text and GUI slider to control the extrusion-brake speed ratio
		scrollbar_ExtrusionBrakeSpeedRatio = mdevice->getGUIEnvironment()->addScrollBar(true, rect<s32>(10, 410, 150, 425), 0, GUI_ID_SCROLLBAR_ExtrusionBrakeSpeedRatio);
		scrollbar_ExtrusionBrakeSpeedRatio->setMin(10);
		scrollbar_ExtrusionBrakeSpeedRatio->setMax(400000);
		scrollbar_ExtrusionBrakeSpeedRatio->setPos((s32)(50 + 50.0 * ((abs(extruderSpeed/brakeGearSpeed) - 0.05) / 0.05)));
		text_ExtrusionBrakeSpeedRatio = mdevice->getGUIEnvironment()->addStaticText(L"Extrusion-brake speed ratio:", rect<s32>(150, 410, 400, 425), false);

		// show extrusion-brake speed ratio as formatted text in interface screen
		double extrusionBrakeSpeedRatio = 0.05 + 0.05 * ((double)(scrollbar_ExtrusionBrakeSpeedRatio->getPos() - 50) / 50.0);
		sprintf(message, "Extrusion-brake speed ratio: %g", extrusionBrakeSpeedRatio);
		text_ExtrusionBrakeSpeedRatio->setText(core::stringw(message).c_str());

		// ..add a text and a button to complete the system configuration
		text_Configuration = mdevice->getGUIEnvironment()->addStaticText(L"Select when configuration completed:", rect<s32>(10, 435, 200, 450), false);
		button_ConfigurationCompleted = mdevice->getGUIEnvironment()->addButton(rect<s32>(210, 435, 350, 450), 0, GUI_ID_BUTTON_ConfigurationCompleted, L"Configuration completed", L"Configuration completed");

		// ..add a button to start the simulation
		button_StartSimulation = mdevice->getGUIEnvironment()->addButton(rect<s32>(10, 460, 100, 475), 0, GUI_ID_BUTTON_StartSimulation, L"Start", L"Start Simulation");
		button_StartSimulation->setEnabled(false);

		// ..add a button to pause the simulation
		button_PauseSimulation = mdevice->getGUIEnvironment()->addButton(rect<s32>(110, 460, 200, 475), 0, GUI_ID_BUTTON_PauseSimulation, L"Pause", L"Pause Simulation");
		button_PauseSimulation->setEnabled(false);

		// ..add a button to stop the simulation
		button_StopSimulation = mdevice->getGUIEnvironment()->addButton(rect<s32>(210, 460, 300, 475), 0, GUI_ID_BUTTON_StopSimulation, L"Stop", L"Stop Simulation");
	}

	bool OnEvent(const SEvent& event) {
		// check if user moved the sliders with mouse..
		if (event.EventType == EET_GUI_EVENT) {
			s32 id = event.GUIEvent.Caller->getID();
			IGUIEnvironment* env = mdevice->getGUIEnvironment();

			switch (event.GUIEvent.EventType) {
			case EGET_SCROLL_BAR_CHANGED:
				if (id == GUI_ID_SCROLLBAR_WireElementLength)  // id of 'wire element length' slider..
				{
					s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					double wireElementLength = 0.005 + 0.005 * (((double)(pos - 50)) / 50.0);
					// set the wire element length (discretization size)
					this->mWireExtruder->SetBeamElementLength(wireElementLength);

					// show wire element length (discretization size) as formatted text in interface screen
					char message[50];
					sprintf(message, "Wire element length [m]: %g", wireElementLength);
					text_WireElementLength->setText(core::stringw(message).c_str());
				}
				if (id == GUI_ID_SCROLLBAR_WireDiameter)  // id of 'wire diameter' slider..
				{
					s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					double wireDiameter = 0.005 + 0.005 * (((double)(pos - 50)) / 50.0);
					// set the wire circular section
					this->mwire->SetAsCircularSection(wireDiameter);

					// show wire diameter as formatted text in interface screen
					char message[50];
					sprintf(message, "Wire diameter [m]: %g", wireDiameter);
					text_WireDiameter->setText(core::stringw(message).c_str());
				}
				if (id == GUI_ID_SCROLLBAR_WireYoungModulus)  // id of 'wire Young elastic modulus' slider..
				{
					s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					double wireYoungModulus = 0.05e9 + 0.05e9 * (((double)(pos - 50)) / 50.0);
					// set the Young modulus of the wire
					this->mwire->SetYoungModulus(wireYoungModulus);

					// show wire Young modulus as formatted text in interface screen
					char message[50];
					sprintf(message, "Wire Young elastic modulus [N/m^2]: %g", wireYoungModulus);
					text_WireYoungModulus->setText(core::stringw(message).c_str());
				}
				if (id == GUI_ID_SCROLLBAR_WireShearModulus)  // id of 'wire shear modulus' slider..
				{
					s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					double wireShearModulus = 0.005e9 + 0.005e9 * (((double)(pos - 50)) / 50.0);
					// set the shear modulus of the wire
					this->mwire->SetGshearModulus(wireShearModulus);

					// show wire shear modulus as formatted text in interface screen
					char message[50];
					sprintf(message, "Wire shear modulus [N/m^2]: %g", wireShearModulus);
					text_WireShearModulus->setText(core::stringw(message).c_str());
				}
				if (id == GUI_ID_SCROLLBAR_WireRayleighDamping)  // id of 'wire Rayleigh damping ratio' slider..
				{
					s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					double wireRayleighDamping = 0.05 + 0.05 * (((double)(pos - 50)) / 50.0);
					// set the Rayleigh damping ratio of the wire
					this->mwire->SetBeamRaleyghDamping(wireRayleighDamping);

					// show wire Rayleigh damping ratio as formatted text in interface screen
					char message[50];
					sprintf(message, "Wire Rayleigh damping ratio: %g", wireRayleighDamping);
					text_WireRayleighDamping->setText(core::stringw(message).c_str());
				}
				if (id == GUI_ID_SCROLLBAR_BrakeGearLobeWidth)  // id of 'brake gear lobe width' slider..
				{
					s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					double brakeGearLobeWidth = 0.005 + 0.005 * (((double)(pos - 50)) / 50.0);

					// show the brake gear lobe width as formatted text in interface screen
					char message[50];
					sprintf(message, "Brake gear lobe width [m]: %g", brakeGearLobeWidth);
					text_BrakeGearLobeWidth->setText(core::stringw(message).c_str());
				}
				if (id == GUI_ID_SCROLLBAR_BrakeGearLobeThickness)  // id of 'brake gear lobe thickness' slider..
				{
					s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					double brakeGearLobeThickness = 0.005 + 0.005 * (((double)(pos - 50)) / 50.0);

					// show the brake gear lobe thickness as formatted text in interface screen
					char message[50];
					sprintf(message, "Brake gear lobe thickness [m]: %g", brakeGearLobeThickness);
					text_BrakeGearLobeThickness->setText(core::stringw(message).c_str());
				}
				if (id == GUI_ID_SCROLLBAR_BrakeGearLobePrimitiveRadius)  // id of 'brake gear lobe primitive radius' slider..
				{
					s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					double brakeGearLobePrimitiveRadius = 0.005 + 0.005 * (((double)(pos - 50)) / 50.0);

					// show the brake gear lobe primitive radius as formatted text in interface screen
					char message[50];
					sprintf(message, "Brake gear lobe primitive radius [m]: %g", brakeGearLobePrimitiveRadius);
					text_BrakeGearLobePrimitiveRadius->setText(core::stringw(message).c_str());
				}
				if (id == GUI_ID_SCROLLBAR_BrakeGearLobeInnerRadius)  // id of 'brake gear lobe inner radius' slider..
				{
					s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					double brakeGearLobeInnerRadius = 0.005 + 0.005 * (((double)(pos - 50)) / 50.0);

					// show the brake gear lobe inner radius as formatted text in interface screen
					char message[50];
					sprintf(message, "Brake gear lobe inner radius [m]: %g", brakeGearLobeInnerRadius);
					text_BrakeGearLobeInnerRadius->setText(core::stringw(message).c_str());
				}
				if (id == GUI_ID_SCROLLBAR_BrakeGearLobeOuterRadius)  // id of 'brake gear lobe outer radius' slider..
				{
					s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					double brakeGearLobeOuterRadius = 0.005 + 0.005 * (((double)(pos - 50)) / 50.0);

					// show the brake gear lobe outer radius as formatted text in interface screen
					char message[50];
					sprintf(message, "Brake gear lobe outer radius [m]: %g", brakeGearLobeOuterRadius);
					text_BrakeGearLobeOuterRadius->setText(core::stringw(message).c_str());
				}
				if (id == GUI_ID_SCROLLBAR_ExtruderSpeed)  // id of 'extruder speed' slider..
				{
					s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					double extruderSpeed = 0.05 + 0.05 * ((double)(pos - 50) / 50.0);

					// set the extruder speed
					this->mWireExtruder->SetExtruderSpeed(extruderSpeed);

					// set the extrusion-brake speed ratio
					double brakeGearSpeed = 0.1 + 0.1 * ((double)(scrollbar_BrakeGearSpeed->getPos() - 50) / 50.0);
					double extrusionBrakeSpeedRatio = abs(extruderSpeed / brakeGearSpeed);
					scrollbar_ExtrusionBrakeSpeedRatio->setPos((s32)(50 + 50.0 * ((extrusionBrakeSpeedRatio - 0.05) / 0.05)));

					// show the extruder speed as formatted text in interface screen
					char message[50];
					sprintf(message, "Wire extruder speed [rad/s]: %g", extruderSpeed);
					text_ExtruderSpeed->setText(core::stringw(message).c_str());
					// show the extrusion brake speed ratio as formatted text in interface screen
					sprintf(message, "Extrusion-brake speed ratio: %g", extrusionBrakeSpeedRatio);
					text_ExtrusionBrakeSpeedRatio->setText(core::stringw(message).c_str());
				}
				if (id == GUI_ID_SCROLLBAR_BrakeGearSpeed)  // id of 'brake gear speed' slider..
				{
					s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					double brakeGearSpeed = 0.1 + 0.1 * ((double)(pos - 50) / 50.0);

					// set the brake gear speed
					if (isConfigurationCompleted)
					{
						auto mLowBrakeGear_SpeedFunction = std::make_shared<ChFunction_Const>(-brakeGearSpeed);
						auto mHighBrakeGear_SpeedFunction = std::make_shared<ChFunction_Const>(brakeGearSpeed);
						this->mLowBrakeGearLink->SetSpeedFunction(mLowBrakeGear_SpeedFunction);
						this->mHighBrakeGearLink->SetSpeedFunction(mHighBrakeGear_SpeedFunction);
					}

					// set the extrusion-brake speed ratio
					double extruderSpeed = 0.05 + 0.05 * ((double)(scrollbar_ExtruderSpeed->getPos() - 50) / 50.0);
					double extrusionBrakeSpeedRatio = abs(extruderSpeed / brakeGearSpeed);
					scrollbar_ExtrusionBrakeSpeedRatio->setPos((s32)(50 + 50.0 * ((extrusionBrakeSpeedRatio - 0.05) / 0.05)));

					// show the brake gear speed as formatted text in interface screen
					char message[50];
					sprintf(message, "Brake gear speed [rad/s]: %g", brakeGearSpeed);
					text_BrakeGearSpeed->setText(core::stringw(message).c_str());
					// show the extrusion brake speed ratio as formatted text in interface screen
					sprintf(message, "Extrusion-brake speed ratio: %g", extrusionBrakeSpeedRatio);
					text_ExtrusionBrakeSpeedRatio->setText(core::stringw(message).c_str());
				}
				if (id == GUI_ID_SCROLLBAR_ExtrusionBrakeSpeedRatio)  // id of 'extrusion-brake speed ratio' slider..
				{
					s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
					double brakeGearSpeed = 0.1 + 0.1 * ((double)(scrollbar_BrakeGearSpeed->getPos() - 50) / 50.0);
					double extrusionBrakeSpeedRatio = 0.05 + 0.05 * ((double)(pos - 50) / 50.0);

					// set the extrusion-brake speed ratio by changing the extrusion speed
					double extruderSpeed = extrusionBrakeSpeedRatio * brakeGearSpeed;
					this->mWireExtruder->SetExtruderSpeed(extruderSpeed);
					scrollbar_ExtruderSpeed->setPos((s32)(50 + 50.0 * ((extruderSpeed - 0.05) / 0.05)));

					// show the extrusion brake speed ratio as formatted text in interface screen
					char message[50];
					sprintf(message, "Wire extruder speed [rad/s]: %g", extruderSpeed);
					text_ExtruderSpeed->setText(core::stringw(message).c_str());
					sprintf(message, "Extrusion-brake speed ratio: %g", extrusionBrakeSpeedRatio);
					text_ExtrusionBrakeSpeedRatio->setText(core::stringw(message).c_str());
				}
				break;
			case EGET_SPINBOX_CHANGED:
				if (id == GUI_ID_SPINBOX_BrakeGearLobeNumber)  // id of 'brake gear lobe number' spinbox..
				{
					s32 lobeNumber = ((IGUISpinBox*)event.GUIEvent.Caller)->getValue();

					// show the brake gear lobe number as formatted text in interface screen
					char message[50];
					sprintf(message, "Brake gear lobe number: %d", lobeNumber);
					text_BrakeGearLobeNumber->setText(core::stringw(message).c_str());
				}
				break;
			case EGET_BUTTON_CLICKED:
				if (id == GUI_ID_BUTTON_ConfigurationCompleted)  // id of the 'configuration done' button..
				{
					char message[50];

					s32 brakeGearLobeNumber = spinbox_BrakeGearLobeNumber->getValue();

					// show brake gear lobe width as formatted text in interface screen
					double lobe_width = 0.005 + 0.005 * ((double)(scrollbar_BrakeGearLobeWidth->getPos() - 50) / 50.0);
					sprintf(message, "Brake gear lobe width [m]: %g", lobe_width);
					text_BrakeGearLobeWidth->setText(core::stringw(message).c_str());

					// show brake gear lobe thickness as formatted text in interface screen
					double lobe_thickness = 0.005 + 0.005 * ((double)(scrollbar_BrakeGearLobeThickness->getPos() - 50) / 50.0);
					sprintf(message, "Brake gear lobe thickness [m]: %g", lobe_thickness);
					text_BrakeGearLobeThickness->setText(core::stringw(message).c_str());

					// show brake gear lobe primitive radius as formatted text in interface screen
					double lobe_primitive_rad = 0.005 + 0.005 * ((double)(scrollbar_BrakeGearLobePrimitiveRadius->getPos() - 50) / 50.0);
					sprintf(message, "Brake gear lobe primitive radius [m]: %g", lobe_primitive_rad);
					text_BrakeGearLobePrimitiveRadius->setText(core::stringw(message).c_str());

					// show brake gear lobe inner radius as formatted text in interface screen
					double lobe_inner_rad = 0.005 + 0.005 * ((double)(scrollbar_BrakeGearLobeInnerRadius->getPos() - 50) / 50.0);
					sprintf(message, "Brake gear lobe inner radius [m]: %g", lobe_inner_rad);
					text_BrakeGearLobeInnerRadius->setText(core::stringw(message).c_str());

					// show brake gear lobe outer radius as formatted text in interface screen
					double lobe_outer_rad = 0.005 + 0.005 * ((double)(scrollbar_BrakeGearLobeOuterRadius->getPos() - 50) / 50.0);
					sprintf(message, "Brake gear lobe outer radius [m]: %g", lobe_outer_rad);
					text_BrakeGearLobeOuterRadius->setText(core::stringw(message).c_str());

					double brakeGearSpeed = 0.1 + 0.1 * ((double)(scrollbar_BrakeGearSpeed->getPos() - 50) / 50.0);

					//
					// Add some obstacles. two rotating lobed gears.
					//
					// Here create two rotating lobed gears, just for fun, that wil trap the 
					// extruded beam. To quickly create them, use the CreateLobedGear() function
					// implemented at the top of this file. 
					// Also, create two simple constant speed motors to rotate the lobed gears.

					ChVector<> gear_centerLOW(0.4, -lobe_primitive_rad, 0);
					ChVector<> gear_centerHI(0.4, lobe_primitive_rad, 0);

					mLowBrakeGear = CreateLobedGear(gear_centerLOW, brakeGearLobeNumber, lobe_width, lobe_primitive_rad,
						lobe_inner_rad, lobe_outer_rad, lobe_thickness, *msystem, msurfacematerial);

					mLowBrakeGearLink = std::make_shared<ChLinkMotorRotationSpeed>();
					mLowBrakeGearLink->Initialize(mLowBrakeGear, mground, ChFrame<>(gear_centerLOW));
					(*msystem).Add(mLowBrakeGearLink);

					auto mgear_speedLOW = std::make_shared<ChFunction_Const>(-brakeGearSpeed); // [rad/s]
					mLowBrakeGearLink->SetSpeedFunction(mgear_speedLOW);

					mHighBrakeGear = CreateLobedGear(gear_centerHI, brakeGearLobeNumber, lobe_width, lobe_primitive_rad,
						lobe_inner_rad, lobe_outer_rad, lobe_thickness, *msystem, msurfacematerial);
					mHighBrakeGear->SetRot(Q_from_AngZ(0.5*CH_C_2PI / brakeGearLobeNumber)); // to phase half step respect to other gear 

					mHighBrakeGearLink = std::make_shared<ChLinkMotorRotationSpeed>();
					mHighBrakeGearLink->Initialize(mHighBrakeGear, mground, ChFrame<>(gear_centerHI));
					(*msystem).Add(mHighBrakeGearLink);

					auto mgear_speedHI = std::make_shared<ChFunction_Const>(brakeGearSpeed); // [rad/s]
					mHighBrakeGearLink->SetSpeedFunction(mgear_speedHI);

					// update brake gear scrollbar
					scrollbar_BrakeGearSpeed->setPos((s32)(50 + 50.0 * ((abs(mHighBrakeGearLink->GetSpeedFunction()->Get_y(0)) - 0.1) / 0.1)));

					// show brake gear speed as formatted text in interface screen
					brakeGearSpeed = 0.1 + 0.1 * ((double)(scrollbar_BrakeGearSpeed->getPos() - 50) / 50.0);
					sprintf(message, "Brake gear speed [rad/s]: %g", brakeGearSpeed);
					text_BrakeGearSpeed->setText(core::stringw(message).c_str());


					isConfigurationCompleted = true;
					isSimulationStarted = false;
					isSimulationPaused = true;

					// ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
					// in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
					// If you need a finer control on which item really needs a visualization proxy in
					// Irrlicht, just use application.AssetBind(myitem); on a per-item basis.

					mapplication->AssetBindAll();

					// ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
					// that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

					mapplication->AssetUpdateAll();

					// Mark completion of system construction
					msystem->SetupInitial();

					button_ConfigurationCompleted->setEnabled(false);
					button_StartSimulation->setEnabled(true);
					button_PauseSimulation->setEnabled(true);
					return isSimulationStarted;
				}
				if (id == GUI_ID_BUTTON_StartSimulation)  // id of the 'start simulation' button..
				{
					isSimulationStarted = true;
					isSimulationPaused = false;
					return isSimulationStarted;
				}
				if (id == GUI_ID_BUTTON_PauseSimulation)  // id of the 'pause simulation' button..
				{
					isSimulationStarted = false;
					isSimulationPaused = true;
					return isSimulationStarted;
				}
				if (id == GUI_ID_BUTTON_StopSimulation)  // id of the 'stop simulation' button..
				{
					isSimulationStarted = false;
					isSimulationPaused = false;
					mdevice->closeDevice();
					return isSimulationStarted;
				}
				break;
			default:
				break;
			}
		}

		return false;
	}

private:
	ChSystemSMC* msystem;
	ChIrrApp* mapplication;
	IrrlichtDevice* mdevice;

	std::shared_ptr<chrono::ChBody> mground;
	std::shared_ptr<chrono::ChMaterialSurfaceSMC> msurfacematerial;
	std::shared_ptr<chrono::fea::ChBeamSectionAdvanced> mwire;
	std::shared_ptr<chrono::fea::ChExtruderBeamEuler> mWireExtruder;
	std::shared_ptr<chrono::ChBody> mLowBrakeGear;
	std::shared_ptr<chrono::ChLinkMotorRotationSpeed> mLowBrakeGearLink;
	std::shared_ptr<chrono::ChBody> mHighBrakeGear;
	std::shared_ptr<chrono::ChLinkMotorRotationSpeed> mHighBrakeGearLink;

	IGUIScrollBar* scrollbar_WireElementLength;
	IGUIStaticText* text_WireElementLength;

	IGUIScrollBar* scrollbar_WireDiameter;
	IGUIStaticText* text_WireDiameter;

	IGUIScrollBar* scrollbar_WireYoungModulus;
	IGUIStaticText* text_WireYoungModulus;

	IGUIScrollBar* scrollbar_WireShearModulus;
	IGUIStaticText* text_WireShearModulus;

	IGUIScrollBar* scrollbar_WireRayleighDamping;
	IGUIStaticText* text_WireRayleighDamping;

	IGUISpinBox* spinbox_BrakeGearLobeNumber;
	IGUIStaticText* text_BrakeGearLobeNumber;

	IGUIScrollBar* scrollbar_BrakeGearLobeWidth;
	IGUIStaticText* text_BrakeGearLobeWidth;

	IGUIScrollBar* scrollbar_BrakeGearLobeThickness;
	IGUIStaticText* text_BrakeGearLobeThickness;

	IGUIScrollBar* scrollbar_BrakeGearLobePrimitiveRadius;
	IGUIStaticText* text_BrakeGearLobePrimitiveRadius;

	IGUIScrollBar* scrollbar_BrakeGearLobeInnerRadius;
	IGUIStaticText* text_BrakeGearLobeInnerRadius;

	IGUIScrollBar* scrollbar_BrakeGearLobeOuterRadius;
	IGUIStaticText* text_BrakeGearLobeOuterRadius;

	IGUIScrollBar* scrollbar_ExtruderSpeed;
	IGUIStaticText* text_ExtruderSpeed;

	IGUIScrollBar* scrollbar_BrakeGearSpeed;
	IGUIStaticText* text_BrakeGearSpeed;

	IGUIScrollBar* scrollbar_ExtrusionBrakeSpeedRatio;
	IGUIStaticText* text_ExtrusionBrakeSpeedRatio;

	IGUIStaticText* text_Configuration;
	IGUIButton* button_ConfigurationCompleted;

	IGUIButton* button_StartSimulation;
	IGUIButton* button_PauseSimulation;
	IGUIButton* button_StopSimulation;
};



int main(int argc, char* argv[]) {
	GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

	// Create a Chrono::Engine physical system
	ChSystemSMC my_system;

	// Here set the inward-outward margins for collision shapes: should make sense in the scale of the model
	collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.001);
	collision::ChCollisionModel::SetDefaultSuggestedMargin(0.002);
	collision::ChCollisionSystemBullet::SetContactBreakingThreshold(0.0001);


	// Create a ground object, useful reference for connecting constraints etc.
	auto mground = std::make_shared<ChBody>();
	mground->SetBodyFixed(true);
	my_system.Add(mground);

	// Create a mesh, that is a container for groups
	// of elements and their referenced nodes.

	auto my_mesh = std::make_shared<ChMesh>();
	my_system.Add(my_mesh);

	// Create a section, i.e. thickness and material properties
	// for beams. This will be shared among some beams.

	auto msection = std::make_shared<ChBeamSectionAdvanced>();

	double wire_diameter = 0.012;
	msection->SetAsCircularSection(wire_diameter);
	msection->SetYoungModulus(0.01e9);  // not exactly a steel wire...
	msection->SetGshearModulus(0.01e9 * 0.3);
	msection->SetBeamRaleyghDamping(0.1);

	// Create the surface material for the contacts; this contains information about friction etc.
	// It is a SMC (penalty) material: interpenetration might happen for low Young stiffness,
	// but unstable simulation might happen for high stiffness, requiring smaller timesteps.


	// option A: Hertz contact force model
	my_system.SetContactForceModel(ChSystemSMC::ContactForceModel::Hertz);
	auto mysurfmaterial = std::make_shared<ChMaterialSurfaceSMC>();
	mysurfmaterial->SetYoungModulus(12e3);  // to adjust heuristically..
	mysurfmaterial->SetRestitution(0.1f);
	mysurfmaterial->SetFriction(0.2f);
	/*
	// Option B: Hooke force model
	my_system.SetContactForceModel(ChSystemSMC::ContactForceModel::Hooke);
	//my_system.UseMaterialProperties(false);
	auto mysurfmaterial = std::make_shared<ChMaterialSurfaceSMC>();
	mysurfmaterial->SetKn(100); // contact normal stiffness
	mysurfmaterial->SetKt(100); // contact tangential stiffness
	mysurfmaterial->SetGn(100);   // contact normal damping
	mysurfmaterial->SetGt(100);   // contact tangential damping
	mysurfmaterial->SetFriction(0.2f);
	*/

	//
	// Add the EXTRUDER
	//

	auto extruder = std::make_shared<ChExtruderBeamEuler>(
		&my_system,                 // the physical system 
		my_mesh,                    // the mesh where to add the beams
		msection,                   // section for created beam
		0.020,                        // beam element length (size of discretization: the smaller, the more precise)
		ChCoordsys<>(ChVector<>(0, 0, 0)), // outlet coordinate system (x axis is the extrusion dir)
		0.04                         // the extrusion speed
		);

	// Enable collision for extruded beam
	extruder->SetContact(mysurfmaterial,  // the NSC material for contact surfaces
		1.15*wire_diameter*0.5  // the radius of the collision spheres at the nodes, (enlarge 15%)
	);

	//
	// Add some other beams 
	//
	// ***NOTE: hack! this is needed because if the extruder starts with 0 beams in the scene, the 
	//    ChVisualizationFEAmesh does not visualize any of the newly generated beams by extrusion. Must be fixed.

	double beam_L = 0.1;

	auto hnode1 = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(-0.4, 0, 0)));
	auto hnode2 = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(-0.4 + beam_L, 0, 0)));
	my_mesh->AddNode(hnode1);
	my_mesh->AddNode(hnode2);

	auto belement1 = std::make_shared<ChElementBeamEuler>();
	belement1->SetNodes(hnode1, hnode2);
	belement1->SetSection(msection);

	my_mesh->AddElement(belement1);
	// Fix a node to ground - the easy way, without constraints
	hnode1->SetFixed(true);


	//
	// Attach a visualization of the FEM mesh.
	//

	auto mvisualizebeamA = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
	mvisualizebeamA->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_ELEM_BEAM_MZ);
	mvisualizebeamA->SetColorscaleMinMax(-0.4, 0.4);
	mvisualizebeamA->SetSmoothFaces(true);
	mvisualizebeamA->SetWireframe(false);
	my_mesh->AddAsset(mvisualizebeamA);

	auto mvisualizebeamC = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
	mvisualizebeamC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);
	mvisualizebeamC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
	mvisualizebeamC->SetSymbolsThickness(0.006);
	mvisualizebeamC->SetSymbolsScale(0.01);
	mvisualizebeamC->SetZbufferHide(false);
	my_mesh->AddAsset(mvisualizebeamC);


	//
	// Add some obstacles. two rotating lobed gears.
	//
	// Here create pointers for the two rotating lobed gears
	// and for two simple constant speed motors used to rotate the lobed gears.

	std::shared_ptr<chrono::ChBody> gearLOW;
	std::shared_ptr<chrono::ChLinkMotorRotationSpeed> mgear_motorLOW;
	std::shared_ptr<chrono::ChBody> gearHI;
	std::shared_ptr<chrono::ChLinkMotorRotationSpeed> mgear_motorHI;


	// Create the Irrlicht visualization (open the Irrlicht device,
	// bind a simple user interface, etc. etc.)
	ChIrrApp application(&my_system, L"Beam continuous extrusion and FEA contacts", core::dimension2d<u32>(800, 600), false, true);

	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	application.AddTypicalLogo();
	application.AddTypicalSky();
	application.AddTypicalLights();
	application.AddTypicalCamera(core::vector3df(-0.1f, 0.2f, -0.2f));

	//
	// USER INTERFACE
	//

	// Create some graphical-user-interface (GUI) items to show on the screen.
	// This requires an event receiver object -see above.
	MyEventReceiver receiver(&my_system, &application, application.GetDevice(), mground, mysurfmaterial, msection, extruder, gearLOW, mgear_motorLOW, gearHI, mgear_motorHI);

	//
	// THE SOFT-REAL-TIME CYCLE
	//

	my_system.SetSolverType(ChSolver::Type::MINRES);
	my_system.SetSolverWarmStarting(true);  // this helps a lot to speedup convergence in this class of problems
	my_system.SetMaxItersSolverSpeed(460);
	my_system.SetMaxItersSolverStab(460);
	my_system.SetTolForce(1e-13);
	auto msolver = std::static_pointer_cast<ChSolverMINRES>(my_system.GetSolver());
	msolver->SetVerbose(false);
	msolver->SetDiagonalPreconditioning(true);

	auto mkl_solver = std::make_shared<ChSolverMKL<>>();
	my_system.SetSolver(mkl_solver);

	application.SetTimestep(0.001);
	application.SetVideoframeSaveInterval(20);

	while (application.GetDevice()->run()) {
		application.BeginScene();

		application.DrawAll();
		ChIrrTools::drawGrid(application.GetVideoDriver(), 0.1, 0.1, 20, 20, CSYSNORM, irr::video::SColor(255, 100, 100, 100), true);

		if (isSimulationStarted && !isSimulationPaused)
		{
			application.DoStep();

			extruder->Update();    //***REMEMBER*** to do this to update the extrusion
		}

		application.EndScene();
	}

	return 0;
}