/*

Copyright (C) University of Oxford, 2005-2012

University of Oxford means the Chancellor, Masters and Scholars of the
University of Oxford, having an administrative office at Wellington
Square, Oxford OX1 2JD, UK.

This file is part of Chaste.

Chaste is free software: you can redistribute it and/or modify it
under the terms of the GNU Lesser General Public License as published
by the Free Software Foundation, either version 2.1 of the License, or
(at your option) any later version.

Chaste is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
License for more details. The offer of Chaste under the terms of the
License is subject to the License being interpreted in accordance with
English Law and subject to any action against the University of Oxford
being under the jurisdiction of the English Courts.

You should have received a copy of the GNU Lesser General Public License
along with Chaste. If not, see <http://www.gnu.org/licenses/>.

*/

#include "PanethCellBoundaryCondition.hpp"
#include "NodeBasedCellPopulation.hpp"
#include "Debug.hpp"

template<unsigned DIM>
PanethCellBoundaryCondition<DIM>::PanethCellBoundaryCondition(AbstractCellPopulation<DIM>* pCellPopulation,
                                                                      double maximumHeight)
    : AbstractCellPopulationBoundaryCondition<DIM>(pCellPopulation),
      mMaximumHeight(maximumHeight)
{
    assert(mMaximumHeight > 0.0);

    if (dynamic_cast<NodeBasedCellPopulation<DIM>*>(this->mpCellPopulation) == NULL)
    {
        EXCEPTION("A NodeBasedCellPopulation must be used with this boundary condition object.");
    }
    if (DIM < 3)
    {
        EXCEPTION("This boundary condition is only implemented in 3D.");
    }
}

template<unsigned DIM>
double PanethCellBoundaryCondition<DIM>::GetMaximumHeightForPanethCells() const
{
    return mMaximumHeight;
}

template<unsigned DIM>
void PanethCellBoundaryCondition<DIM>::ImposeBoundaryCondition(const std::map<Node<DIM>*, c_vector<double, DIM> >& rOldLocations)
{
    // Iterate over the cell population
    for (typename AbstractCellPopulation<DIM>::Iterator cell_iter = this->mpCellPopulation->Begin();
         cell_iter != this->mpCellPopulation->End();
         ++cell_iter)
    {
    	// Firstly check if the cell is a Paneth (or stem) cell or not, then if it has moved too far up the crypt
    	if (cell_iter->GetCellProliferativeType()->template IsType<PanethCellProliferativeType>()
    			|| cell_iter->GetCellProliferativeType()->template IsType<StemCellProliferativeType>())
		{
            // Get index of node associated with cell
            unsigned node_index = this->mpCellPopulation->GetLocationIndexUsingCell(*cell_iter);

            // Get pointer to this node
            Node<DIM>* p_node = this->mpCellPopulation->GetNode(node_index);

    		c_vector<double,DIM> cell_location = this->mpCellPopulation->GetLocationOfCellCentre(*cell_iter);

    		// If the cell has moved beyond the limit for Paneth cells, put it back
    		if (cell_location[DIM-1] > mMaximumHeight)
    		{
    			p_node->rGetModifiableLocation()[DIM-1] = mMaximumHeight;
    		}

    		assert(p_node->rGetLocation()[DIM-1] <= mMaximumHeight);

		}
    }
}

template<unsigned DIM>
bool PanethCellBoundaryCondition<DIM>::VerifyBoundaryCondition()
{
    bool boundary_condition_satisfied = true;

    /*
     * Here we verify that the boundary condition is still satisfied by checking that no paneth cells lie
	 * above the maximum height defined
     */
    for (typename AbstractCellPopulation<DIM>::Iterator cell_iter = this->mpCellPopulation->Begin();
         cell_iter != this->mpCellPopulation->End();
         ++cell_iter)
    {

    	if (cell_iter->GetCellProliferativeType()->template IsType<PanethCellProliferativeType>())
		{
			// Get index of node associated with cell
			unsigned node_index = this->mpCellPopulation->GetLocationIndexUsingCell(*cell_iter);

			// Get pointer to this node
			Node<DIM>* p_node = this->mpCellPopulation->GetNode(node_index);

			// If this node sits above the threshold height for paneth cells, return false
			if (p_node->rGetLocation()[DIM-1] > mMaximumHeight)
			{
				boundary_condition_satisfied = false;
	    		c_vector<double,DIM> cell_location = this->mpCellPopulation->GetLocationOfCellCentre(*cell_iter);
				//PRINT_5_VARIABLES(node_index, cell_location[0],cell_location[1],cell_location[2], mMaximumHeight);
				break;
			}
		}
    }

    return boundary_condition_satisfied;
}

template<unsigned DIM>
void PanethCellBoundaryCondition<DIM>::OutputCellPopulationBoundaryConditionParameters(out_stream& rParamsFile)
{
    *rParamsFile << "\t\t\t<MaximumHeight>" << mMaximumHeight << "</MaximumHeight>\n";

    // Call method on direct parent class
    AbstractCellPopulationBoundaryCondition<DIM>::OutputCellPopulationBoundaryConditionParameters(rParamsFile);
}

/////////////////////////////////////////////////////////////////////////////
// Explicit instantiation
/////////////////////////////////////////////////////////////////////////////

template class PanethCellBoundaryCondition<1>;
template class PanethCellBoundaryCondition<2>;
template class PanethCellBoundaryCondition<3>;

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(PanethCellBoundaryCondition)
