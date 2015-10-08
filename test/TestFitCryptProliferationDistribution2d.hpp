
#ifndef TESTCRYPTPROLIFERATIONDISTRIBUTION_HPP_
#define TESTCRYPTPROLIFERATIONDISTRIBUTION_HPP_

#include <cxxtest/TestSuite.h>

// Must be included before any other cell_based headers
#include "CellBasedSimulationArchiver.hpp"
#include "AbstractCellBasedTestSuite.hpp"
#include "CellBasedEventHandler.hpp"

#include "CryptSimulation2d.hpp"
#include "VolumeTrackingModifier.hpp"

#include "CryptCellsGenerator.hpp"
#include "CellsGenerator.hpp"

#include "CylindricalHoneycombMeshGenerator.hpp"

#include "CryptCellCycleModel.hpp"

#include "PanethCellProliferativeType.hpp"
#include "TransitCellProliferativeType.hpp"

#include "CellAgesWriter.hpp"
#include "CellVolumesWriter.hpp"
#include "CellProliferativeTypesWriter.hpp"
#include "CellMutationStatesWriter.hpp"
#include "NodeVelocityWriter.hpp"

#include "GeneralisedLinearSpringForce.hpp"
#include "DifferentialAdhesionSpringForce.hpp"
#include "RepulsionForce.hpp"
#include "CellRetainerForce.hpp"

#include "CryptSimulationBoundaryCondition.hpp"
#include "CryptGeometryBoundaryCondition3d.hpp"

#include "MeshBasedCellPopulationWithGhostNodes.hpp"
#include "NodeBasedCellPopulation.hpp"


#include "SloughingCellKiller.hpp"
#include "PlaneBasedCellKiller.hpp"

#include "PetscSetupAndFinalize.hpp"

#include "Debug.hpp"

class TestFitCryptProliferationDistribution : public AbstractCellBasedTestSuite
{
private:

    double mLastStartTime;
    void setUp()
    {
        mLastStartTime = std::clock();
        AbstractCellBasedTestSuite::setUp();
    }
    void tearDown()
    {
        double time = std::clock();
        double elapsed_time = (time - mLastStartTime)/(CLOCKS_PER_SEC);
        std::cout << "Elapsed time: " << elapsed_time << std::endl;
        AbstractCellBasedTestSuite::tearDown();
    }

public:


    // Run with command ./projects/CryptProliferation2013/build/optimised/TestFitCryptProliferationDistribution2dRunner -end_time 1 -CCM 1 -min 0 -max 1 -num_sweeps 5 -CI -min_CI 0 -max_CI 1 -num_CI_sweeps 5

    void Test2DCrypt() throw (Exception)
    {


    	// Model setup


        /*
         * 1 - Pedigree
         * 2 - Spatial Wnt
         * 3 - Spatial Wnt at birth
         * 4 - Mutant
         */
    	TS_ASSERT(CommandLineArguments::Instance()->OptionExists("-CCM"));
    	unsigned cell_proliferation_model = (unsigned) atof(CommandLineArguments::Instance()->GetStringCorrespondingToOption("-CCM").c_str());
    	assert(cell_proliferation_model==1 || cell_proliferation_model==2 || cell_proliferation_model==3  || cell_proliferation_model==4);

    	bool contact_inhibition = CommandLineArguments::Instance()->OptionExists("-CI");

    	bool wnt_dependent_ccd = CommandLineArguments::Instance()->OptionExists("-WDCCD");


    	// Sim and sweep Params
    	TS_ASSERT(CommandLineArguments::Instance()->OptionExists("-end_time"));
    	double end_time = atof(CommandLineArguments::Instance()->GetStringCorrespondingToOption("-end_time").c_str());

    	// Param Sweep
    	TS_ASSERT(CommandLineArguments::Instance()->OptionExists("-min"));
        double min_param = atof(CommandLineArguments::Instance()->GetStringCorrespondingToOption("-min").c_str());

        TS_ASSERT(CommandLineArguments::Instance()->OptionExists("-max"));
        double max_param = atof(CommandLineArguments::Instance()->GetStringCorrespondingToOption("-max").c_str());

        assert(min_param>=0);
        if (cell_proliferation_model ==2 || cell_proliferation_model==3) // i.e wnt dependent
        {
        	assert (max_param<=1);
		}

        TS_ASSERT(CommandLineArguments::Instance()->OptionExists("-num_sweeps"));
    	double num_sweeps = atof(CommandLineArguments::Instance()->GetStringCorrespondingToOption("-num_sweeps").c_str());

    	// CI Sweep
    	TS_ASSERT(CommandLineArguments::Instance()->OptionExists("-min_CI"));
        double min_CI_param = atof(CommandLineArguments::Instance()->GetStringCorrespondingToOption("-min_CI").c_str());

        TS_ASSERT(CommandLineArguments::Instance()->OptionExists("-max_CI"));
        double max_CI_param = atof(CommandLineArguments::Instance()->GetStringCorrespondingToOption("-max_CI").c_str());

    	TS_ASSERT(CommandLineArguments::Instance()->OptionExists("-num_CI_sweeps"));
    	double num_CI_sweeps = atof(CommandLineArguments::Instance()->GetStringCorrespondingToOption("-num_CI_sweeps").c_str());



//		unsigned cell_proliferation_model = 2;
//		bool contact_inhibition = true;
//		bool wnt_dependent_ccd = false;
//
//		double end_time = 500;
//		double min_param = 0;
//		double max_param = 1;
//		double num_sweeps = 5;
//		double min_CI_param = 0;
//		double max_CI_param = 1;
//		double num_CI_sweeps = 5;




//		unsigned max_index = 4; // Number of things to sweep over
//		double max_wnt_thresh = 1.0;
//		double min_wnt_thresh = 0.0;
//
//		unsigned min_generations = 1; // As 0 means all differentiated in Steady State
//		unsigned max_generations = 3;

    	// Set up cells
    	unsigned cells_across = 16;
		double crypt_width = 16.0;
		double crypt_length = 12;
		unsigned cells_up = 1;
		unsigned thickness_of_ghost_layer = 2;
		// Choosing same dimensions as for halted migration paper


         for (unsigned index = 0; index<=num_sweeps;   index ++)
         {
        	double param = min_param + (double)index / double(num_sweeps) * (max_param-min_param);

			PRINT_3_VARIABLES(cell_proliferation_model,
								index,
								param);

			// Extra code to calculate the generation if Model 1
			unsigned upper_max_transit_generations = (unsigned)ceil(param);
			unsigned lower_max_transit_generations = (unsigned)floor(param);
			double prob_of_upper_max_transit_generations = param - floor(param);

//				PRINT_4_VARIABLES(max_transit_generations,
//								  lower_max_transit_generations,
//								  upper_max_transit_generations,
//								  prob_of_upper_max_transit_generations);

			//Sweep over CI param
			for (unsigned CIindex = 0; CIindex<=num_CI_sweeps;   CIindex ++)
			{
				double CIparam = min_CI_param + (double)CIindex / double(num_CI_sweeps) * (max_CI_param-min_CI_param);
				PRINT_2_VARIABLES(CIindex,CIparam);

				// Create some starter nodes
				CylindricalHoneycombMeshGenerator generator(cells_across, cells_up,thickness_of_ghost_layer, crypt_width/cells_across);
			    Cylindrical2dMesh* p_mesh = generator.GetCylindricalMesh();

			    std::vector<unsigned> location_indices = generator.GetCellLocationIndices();

				// Create cells
				std::vector<CellPtr> cells;

				CellsGenerator<CryptCellCycleModel, 2> cells_generator;
				cells_generator.GenerateBasicRandom(cells, location_indices.size());

				//Change properties of the ccm
				for (unsigned cell_index= 0;  cell_index<cells.size(); cell_index++)
				{
					  /*
					   * Specify CCM
					   */
					  dynamic_cast<CryptCellCycleModel*>(cells[cell_index]->GetCellCycleModel())->SetCellProliferationModel(cell_proliferation_model);
					  dynamic_cast<CryptCellCycleModel*>(cells[cell_index]->GetCellCycleModel())->SetIsContactInhibitionCellCycleDuration((bool)contact_inhibition);
					  dynamic_cast<CryptCellCycleModel*>(cells[cell_index]->GetCellCycleModel())->SetIsWntDependentCellCycleDuration((bool)wnt_dependent_ccd);

					  /*
					   * Set some default CCD parameters So total CCM is U[10,14] and (U[22,26] at base if variable)
					   */

					  dynamic_cast<CryptCellCycleModel*>(cells[cell_index]->GetCellCycleModel())->SetMDuration(4.0);
					  dynamic_cast<CryptCellCycleModel*>(cells[cell_index]->GetCellCycleModel())->SetSDuration(4.0);
					  dynamic_cast<CryptCellCycleModel*>(cells[cell_index]->GetCellCycleModel())->SetG2Duration(2.0);
					  dynamic_cast<CryptCellCycleModel*>(cells[cell_index]->GetCellCycleModel())->SetTransitCellG1Duration(2.0);  // so total CCM is U[10,14] at threshold
					  dynamic_cast<CryptCellCycleModel*>(cells[cell_index]->GetCellCycleModel())->SetStemCellG1Duration(14.0);  // so total CCM is U[10,14] at base

					  /*
					   * Threshold and Generation specific parameters
					   */
				      if (cell_proliferation_model ==1 ) // i.e Pedigree dependent
				      {
				    	  dynamic_cast<CryptCellCycleModel*>(cells[cell_index]->GetCellCycleModel())->SetWntThreshold(1.0);
				    	  if (RandomNumberGenerator::Instance()->ranf()<prob_of_upper_max_transit_generations)
				    	  {
				    		  dynamic_cast<CryptCellCycleModel*>(cells[cell_index]->GetCellCycleModel())->SetMaxTransitGenerations(upper_max_transit_generations); // Mutant = MAX_UNSIGNED
				    	  }
				    	  else
				    	  {
				    		  dynamic_cast<CryptCellCycleModel*>(cells[cell_index]->GetCellCycleModel())->SetMaxTransitGenerations(lower_max_transit_generations); // Mutant = MAX_UNSIGNED
				    	  }
				      }
				      else
				      {
				    	  dynamic_cast<CryptCellCycleModel*>(cells[cell_index]->GetCellCycleModel())->SetWntThreshold(param);
				    	  dynamic_cast<CryptCellCycleModel*>(cells[cell_index]->GetCellCycleModel())->SetMaxTransitGenerations(UINT_MAX);
  				      }

    				  /*
					   * Contact Inhibition specific parameters (Mutant, same CI)
					   */
					  dynamic_cast<CryptCellCycleModel*>(cells[cell_index]->GetCellCycleModel())->SetEquilibriumVolume(3.0/4.0*sqrt(3.0));
					  dynamic_cast<CryptCellCycleModel*>(cells[cell_index]->GetCellCycleModel())->SetQuiescentVolumeFraction(CIparam);
					  dynamic_cast<CryptCellCycleModel*>(cells[cell_index]->GetCellCycleModel())->SetMutantQuiescentVolumeFraction(CIparam);

				}

				// Create cell population
				MeshBasedCellPopulationWithGhostNodes<2> crypt(*p_mesh, cells, location_indices);

				// Output data
				crypt.AddCellWriter<CellAgesWriter>();
				crypt.AddCellWriter<CellVolumesWriter>();
				crypt.AddCellWriter<CellProliferativeTypesWriter>();
				crypt.AddCellWriter<CellMutationStatesWriter>();
				//crypt.AddPopulationWriter<NodeVelocityWriter>();


				// Create an instance of a Wnt concentration NOTE DO THIS BEFORE THE SIMULATION OTHERWISE CELLS CANT INITIALISE
				WntConcentration<2>::Instance()->SetType(LINEAR);
				WntConcentration<2>::Instance()->SetCellPopulation(crypt);
				WntConcentration<2>::Instance()->SetCryptLength(crypt_length);


				// Create a crypt simulator to lock stem cells in place
				CryptSimulation2d simulator(crypt);

				simulator.SetOutputDivisionLocations(true);
				simulator.SetDt(1.0/120.0);
				simulator.SetSamplingTimestepMultiple(120);
			    simulator.UseJiggledBottomCells();
				simulator.SetEndTime(end_time);

				// Add Volume Tracking Modifier
				MAKE_PTR(VolumeTrackingModifier<2>, p_modifier);
				simulator.AddSimulationModifier(p_modifier);

				//Create output directory
				std::stringstream out;
				if (cell_proliferation_model == 1)
				{
					out << "FitCCM_2D_"<< cell_proliferation_model << "_CI_" << contact_inhibition << "_WDCCD_" << wnt_dependent_ccd << "_MaxGen_" << param << "_CIthresh_" << CIparam;

				}
				else
				{
					out << "FitCCM_2D_"<< cell_proliferation_model << "_CI_" << contact_inhibition << "_WDCCD_" << wnt_dependent_ccd << "_WntThresh_" << param << "_CIthresh_" << CIparam;
				}
				std::string output_directory = "CryptProlifFit/" +  out.str();
				simulator.SetOutputDirectory(output_directory);

		        // Create a force law and pass it to the simulation
		        MAKE_PTR(GeneralisedLinearSpringForce<2>, p_force);
		        p_force->SetMeinekeSpringStiffness(30.0); //normally 15.0;
		        //p_force->SetCutOffLength(1.5);
		        simulator.AddForce(p_force);

		        // Create cell killer and pass in to crypt simulation
		        MAKE_PTR_ARGS(SloughingCellKiller<2>, p_cell_killer,(&crypt, crypt_length));
		        simulator.AddCellKiller(p_cell_killer);

				// Run simulation
				simulator.Solve();

				// Extra Gubbins to get to loop: this is usually done by the SetUp and TearDown methods
				WntConcentration<2>::Instance()->Destroy();
				SimulationTime::Instance()->Destroy();
				SimulationTime::Instance()->SetStartTime(0.0);
			}
         }
	}
};

#endif /*TESTCRYPTPROLIFERATIONDISTRIBUTION_HPP_*/
