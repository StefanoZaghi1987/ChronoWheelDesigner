// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Alessandro Tasora
// =============================================================================
//
// Create a falling and colliding cable using FEA module (FEA tutorial n.2)
//
// This model is made with N elements of ChElementBeamEuler type. They are
// added to a ChMesh and then the first cable is connected to the absolute
// reference using a joint.
//
// A simple ChContactSurfaceNodeCloud is used to provide collision against
// the floor.
//
// The cable falls under the action of gravity alone, acting in the negative
// Y (up) direction.
//
// The simulation is animated with Irrlicht.
//
// =============================================================================

#include <cstdio>
#include <cmath>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_irrlicht/ChIrrApp.h"

#include "chrono_fea/ChElementBeamEuler.h"
#include "chrono_fea/ChBuilderBeam.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"
#include "chrono_fea/ChContactSurfaceMesh.h"
#include "chrono_fea/ChContactSurfaceNodeCloud.h"

#include "chrono/timestepper/ChTimestepper.h"
#include "chrono_mkl/ChSolverMKL.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::fea;
using namespace irr;

int main(int argc, char* argv[]) {

	// 1. Create the physical system that will handle all finite elements and constraints.

	//    NOTE that we need contact in FEA, so we use the ChSystemSMC, that uses SMC penalty in contacts
	ChSystemSMC system;
	/*system.Set_G_acc(ChVector<>(9.81, 0, 0));*/
	system.Set_G_acc(ChVector<>(0, 0, 0));


	// 2. Create the mesh that will contain the finite elements, and add it to the system

	auto mesh = std::make_shared<ChMesh>();
	system.Add(mesh);


	// 3. Create a material for the beam finite elements.

	//    Note that each FEA element type requires some corresponding
	//    type of material. ChElemetBeamEuler require a ChBeamSectionAdvanced material.

	auto beam_material = std::make_shared<ChBeamSectionAdvanced>();
	beam_material->SetAsRectangularSection(0.012, 0.025);
	beam_material->SetYoungModulus(0.01e9);
	beam_material->SetGshearModulus(0.01e9 * 0.3);
	beam_material->SetBeamRaleyghDamping(0.01);


	// 4. Create the nodes

	//    - We use a simple for() loop to create nodes along the cable.
	//    - Nodes for ChElemetBeamEuler must be of ChNodeFEAxyzrot class;
	//      i.e. each node has coordinates of type: {position, rotation},
	//      where X axis of rotated system is the direction of the beam, 
	//      Y and Z are the section plane.

	std::vector<std::shared_ptr<ChNodeFEAxyzrot> > beam_nodes;
	std::vector<std::shared_ptr<ChBodyEasyBox> > beam_nodes_boxes;

	double length = 1.2;  // beam length, in meters;
	int N_nodes = 16;
	for (int in = 0; in < N_nodes; ++in) {
		// i-th node position
		ChVector<> position(length * (in / double(N_nodes - 1)),		// node position, x
			0.2,														// node position, y
			0);															// node position, z

		// create the node
		auto node = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(position));

		// add it to mesh
		mesh->AddNode(node);

		// add it to the auxiliary beam_nodes
		beam_nodes.push_back(node);

		// make a box and connect it
		auto mbox = std::make_shared<ChBodyEasyBox>(0.002, 0.004, 0.004, 1000);
		mbox->SetPos(node->GetPos());

		// add the box to the auxiliary beam_nodes_boxes
		beam_nodes_boxes.push_back(mbox);

		// add the box to the system
		system.Add(mbox);

		// lock the node to the corresponding box body element
		auto constraint_pos = std::make_shared<ChLinkMateSpherical>();
		constraint_pos->Initialize(
			node,				// node to constraint
			mbox,				// body to constraint
			false,				// false: next 2 pos are in absolute coords, true: in relative coords
			node->GetPos(),		// sphere ball pos 
			mbox->GetPos()		// sphere cavity pos
		);
		system.Add(constraint_pos);
	}


	// 5. Create the elements

	for (int ie = 0; ie < N_nodes - 1; ++ie) {
		// create the element
		auto element = std::make_shared<ChElementBeamEuler>();

		// set the connected nodes (pick two consecutive nodes in our beam_nodes container)
		element->SetNodes(beam_nodes[ie], beam_nodes[ie + 1]);

		// set the material
		element->SetSection(beam_material);

		// add it to mesh
		mesh->AddElement(element);

		/*auto link_actuatorFork = std::make_shared<ChLinkLinActuator>();
		link_actuatorFork->Initialize(beam_nodes_boxes[ie], beam_nodes_boxes[ie + 1], false,
			ChCoordsys<>(beam_nodes[ie]->GetPos(), QUNIT),
			ChCoordsys<>(beam_nodes[ie + 1]->GetPos(), QUNIT));
		system.AddLink(link_actuatorFork);

		auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(link_actuatorFork->Get_dist_funct());
		mfun->Set_yconst(0.1);*/
	}


	// 6. Add constraints

	//    - Constraints can be applied to FEA nodes
	//    - For the ChNodeFEAxyzrot one can use all constraints
	//      of the ChMate class


	// 7. Add a collision mesh to the skin of the finite element mesh

	//    - Create a ChMaterialSurfaceSMC , it must be assigned to FEA 
	//      meshes and rigid bodies. The ChSystemSMC requires it!
	//    - Create a ChContactSurfaceNodeCloud and add to the FEA mesh.
	//      This is the easiest representation of a FEA contact surface: it
	//      simply creates contact spheres per each node. So, no edge-edge cases
	//      can be detected between elements though, but it is enough for
	//      dense finite elements meshes that collide with large objects.

	// Create a surface material to be shared with some objects
	auto mysurfmaterial = std::make_shared<ChMaterialSurfaceSMC>();
	mysurfmaterial->SetYoungModulus(2e4);
	mysurfmaterial->SetFriction(0.3f);
	mysurfmaterial->SetRestitution(0.2f);
	mysurfmaterial->SetAdhesion(0);

	// Create the contact surface and add to the mesh
	auto mcontactcloud = std::make_shared<ChContactSurfaceNodeCloud>();
	mesh->AddContactSurface(mcontactcloud);

	// Must use this to 'populate' the contact surface use larger point size to match beam section radius
	mcontactcloud->AddAllNodes(0.01);

	// Use our SMC surface material properties 
	mcontactcloud->SetMaterialSurface(mysurfmaterial);


	// 8. Create a collision plane, as a huge box

	auto plane = std::make_shared<ChBodyEasyBox>(
		0.2, 4, 4,  // x,y,z size
		1000,       // density
		true,       // visible
		true        // collide
		);

	system.Add(plane);
	plane->SetBodyFixed(true);
	plane->SetPos(ChVector<>(2.6, -0.1, 0));
	// Use our SMC surface material properties 
	plane->SetMaterialSurface(mysurfmaterial);

	// 8. Create the floor, as a huge box

	auto floor = std::make_shared<ChBodyEasyBox>(
		4, 0.2, 4,  // x,y,z size
		1000,       // density
		true,       // visible
		true        // collide
		);

	system.Add(floor);
	floor->SetBodyFixed(true);
	floor->SetPos(ChVector<>(0, -1.1, 0));
	// Use our SMC surface material properties 
	floor->SetMaterialSurface(mysurfmaterial);



	// 9. Make the finite elements visible in the 3D view

	//   - FEA fisualization can be managed via an easy
	//     ChVisualizationFEAmesh helper class.
	//     (Alternatively you could bypass this and output .dat
	//     files at each step, ex. for VTK or Matalb postprocessing)
	//   - This will automatically update a triangle mesh (a ChTriangleMeshShape
	//     asset that is internally managed) by setting proper
	//     coordinates and vertex colours as in the FEA elements.
	//   - Such triangle mesh can be rendered by Irrlicht or POVray or whatever
	//     postprocessor that can handle a coloured ChTriangleMeshShape).
	//   - Do not forget AddAsset() at the end!

	auto mvisualizebeamA = std::make_shared<ChVisualizationFEAmesh>(*(mesh.get()));
	mvisualizebeamA->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_ANCF_BEAM_AX);
	mvisualizebeamA->SetColorscaleMinMax(-0.005, 0.005);
	mvisualizebeamA->SetSmoothFaces(true);
	mvisualizebeamA->SetWireframe(false);
	mesh->AddAsset(mvisualizebeamA);

	auto mvisualizebeamC = std::make_shared<ChVisualizationFEAmesh>(*(mesh.get()));
	mvisualizebeamC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);
	mvisualizebeamC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
	mvisualizebeamC->SetSymbolsThickness(0.006);
	mvisualizebeamC->SetSymbolsScale(0.005);
	mvisualizebeamC->SetZbufferHide(false);
	mesh->AddAsset(mvisualizebeamC);


	// 10. Configure the solver and timestepper

	//    - the default SOLVER_SOR of Chrono is not able to manage stiffness matrices
	//      as required by FEA! we must switch to a different solver.
	//    - We pick the MKL solver

	// Change solver to MKL
	auto mkl_solver = std::make_shared<ChSolverMKL<>>();
	mkl_solver->SetSparsityPatternLock(false);
	//system.SetSolverWarmStarting(true);  // this helps a lot to speedup convergence in this class of problems
	//system.SetMaxItersSolverSpeed(200);
	//system.SetMaxItersSolverStab(200);
	//system.SetTolForce(1e-10);
	system.SetSolver(mkl_solver);

	// WARNING: due to known issues on MKL Pardiso, if CSR matrix is used, sparsity pattern lock should be put OFF
	// Look at ChCSMatrix::SetElement comments to further details.
	system.Update();

	// Change type of integrator:
	system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);  // fast, less precise
	//system.SetTimestepperType(chrono::ChTimestepper::Type::HHT);  // precise,slower, might iterate each step

	// if later you want to change integrator settings:
	if (auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(system.GetTimestepper()))
	{
		mystepper->SetAlpha(-0.2);
		mystepper->SetMaxiters(2);
		mystepper->SetAbsTolerances(1e-6);
	}


	// 11. Prepare visualization with Irrlicht
	//    Note that Irrlicht uses left-handed frames with Y up.

	// Create the Irrlicht application and set-up the camera.
	ChIrrApp* application = new ChIrrApp(&system,                            // pointer to the mechanical system
		L"FEA cable collide demo",          // title of the Irrlicht window
		core::dimension2d<u32>(1024, 768),  // window dimension (width x height)
		false,                              // use full screen?
		true,                               // enable stencil shadows?
		true);                              // enable antialiasing?

	application->AddTypicalLogo();
	application->AddTypicalSky();
	application->AddTypicalLights();
	application->AddTypicalCamera(core::vector3df(0.1f, 0.2f, -2.0f),  // camera location
		core::vector3df(0.0f, 0.0f, 0.0f));  // "look at" location

											 // Enable drawing of contacts
	application->SetContactsDrawMode(ChIrrTools::CONTACT_FORCES);
	application->SetSymbolscale(0.1);

	// Let the Irrlicht application convert the visualization assets.
	application->AssetBindAll();
	application->AssetUpdateAll();


	// 12. Perform the simulation.

	// Specify the step-size.
	application->SetTimestep(0.001);
	application->SetTryRealtime(false);

	// Mark completion of system construction
	system.SetupInitial();

	while (application->GetDevice()->run()) {
		// Initialize the graphical scene.
		application->BeginScene();

		// Render all visualization objects.
		application->DrawAll();

		// Draw an XZ grid at the global origin to add in visualization.
		ChIrrTools::drawGrid(application->GetVideoDriver(), 0.1, 0.1, 20, 20,
			ChCoordsys<>(ChVector<>(0, -1, 0), Q_from_AngX(CH_C_PI_2)),
			video::SColor(255, 80, 100, 100), true);

		// Draw an YZ grid at the global origin to add in visualization.
		ChIrrTools::drawGrid(application->GetVideoDriver(), 0.1, 0.1, 20, 20,
			ChCoordsys<>(ChVector<>(2.5, 0, 0), Q_from_AngY(CH_C_PI_2)),
			video::SColor(255, 80, 100, 100), true);

		// Advance simulation by one step.
		application->DoStep();

		// Finalize the graphical scene.
		application->EndScene();
	}

	return 0;
}