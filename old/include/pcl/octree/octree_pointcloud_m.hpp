/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 */

#pragma once

#include <pcl/octree/octree_base.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

namespace pcl
{
  namespace octree
  {

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Octree pointcloud class
     *  \note Octree implementation for pointclouds. Only indices are stored by the octree leaf nodes (zero-copy).
     *  \note The octree pointcloud class needs to be initialized with its voxel resolution. Its bounding box is automatically adjusted
     *  \note according to the pointcloud dimension or it can be predefined.
     *  \note Note: The tree depth equates to the resolution and the bounding box dimensions of the octree.
     *  \note
     *  \note typename: PointT: type of point used in pointcloud
     *  \note typename: LeafContainerT:  leaf node container (
     *  \note typename: BranchContainerT:  branch node container
     *  \note typename: OctreeT: octree implementation ()
     *  \ingroup octree
     *  \author Julius Kammerl (julius@kammerl.de)
     */
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafContainerT = OctreeContainerPointIndices,
        typename BranchContainerT = OctreeContainerEmpty,
        typename OctreeT = OctreeBase<LeafContainerT, BranchContainerT> >

    class MyOctreePointCloud : public OctreeT
    {
      public:
        typedef OctreeT Base;

        typedef typename OctreeT::LeafNode LeafNode;
        typedef typename OctreeT::BranchNode BranchNode;

        /** \brief Octree pointcloud constructor.
         * \param[in] resolution_arg octree resolution at lowest octree level
         */
        MyOctreePointCloud (const double resolution_arg);

        /** \brief Empty deconstructor. */
        
        ~MyOctreePointCloud ();

        // public typedefs
        typedef boost::shared_ptr<std::vector<int> > IndicesPtr;
        typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;

        typedef pcl::PointCloud<PointT> PointCloud;
        typedef boost::shared_ptr<PointCloud> PointCloudPtr;
        typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

        // public typedefs for single/double buffering
        typedef MyOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeBase<LeafContainerT> > SingleBuffer;
       // typedef MyOctreePointCloud<PointT, LeafContainerT, BranchContainerT, Octree2BufBase<LeafContainerT> > DoubleBuffer;

        // Boost shared pointers
        typedef boost::shared_ptr<MyOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT> > Ptr;
        typedef boost::shared_ptr<const MyOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT> > ConstPtr;

        // Eigen aligned allocator
        typedef std::vector<PointT, Eigen::aligned_allocator<PointT> > AlignedPointTVector;
        typedef std::vector<PointXYZ, Eigen::aligned_allocator<PointXYZ> > AlignedPointXYZVector;

        /** \brief Provide a pointer to the input data set.
         * \param[in] cloud_arg the const boost shared pointer to a PointCloud message
         * \param[in] indices_arg the point indices subset that is to be used from \a cloud - if 0 the whole point cloud is used
         */
        inline void setInputCloud (const PointCloudConstPtr &cloud_arg,
            const IndicesConstPtr &indices_arg = IndicesConstPtr ())
        {
          input_ = cloud_arg;
          indices_ = indices_arg;
        }

        /** \brief Get a pointer to the vector of indices used.
         * \return pointer to vector of indices used.
         */
        inline IndicesConstPtr const getIndices () const
        {
          return (indices_);
        }

        /** \brief Get a pointer to the input point cloud dataset.
         * \return pointer to pointcloud input class.
         */
        inline PointCloudConstPtr getInputCloud () const
        {
          return (input_);
        }

        /** \brief Set the search epsilon precision (error bound) for nearest neighbors searches.
         * \param[in] eps precision (error bound) for nearest neighbors searches
         */
        inline void setEpsilon (double eps)
        {
          epsilon_ = eps;
        }

        /** \brief Get the search epsilon precision (error bound) for nearest neighbors searches. */
        inline double getEpsilon () const
        {
          return (epsilon_);
        }

        /** \brief Set/change the octree voxel resolution
         * \param[in] resolution_arg side length of voxels at lowest tree level
         */
        inline void setResolution (double resolution_arg)
        {
          // octree needs to be empty to change its resolution
          assert( this->leaf_count_ == 0);

          resolution_ = resolution_arg;

          getKeyBitSize ();
        }

        /** \brief Get octree voxel resolution
         * \return voxel resolution at lowest tree level
         */
        inline double getResolution () const
        {
          return (resolution_);
        }

        /** \brief Get the maximum depth of the octree.
         *  \return depth_arg: maximum depth of octree
         * */
        inline unsigned int getTreeDepth () const
        {
          return this->octree_depth_;
        }

        /** \brief Add points from input point cloud to octree. */
        void
        addPointsFromInputCloud ();

        /** \brief Add point at given index from input point cloud to octree. Index will be also added to indices vector.
         * \param[in] point_idx_arg index of point to be added
         * \param[in] indices_arg pointer to indices vector of the dataset (given by \a setInputCloud)
         */
        void
        addPointFromCloud (const int point_idx_arg, IndicesPtr indices_arg);

        /** \brief Add point simultaneously to octree and input point cloud.
         *  \param[in] point_arg point to be added
         *  \param[in] cloud_arg pointer to input point cloud dataset (given by \a setInputCloud)
         */
        void
        addPointToCloud (const PointT& point_arg, PointCloudPtr cloud_arg);

        /** \brief Add point simultaneously to octree and input point cloud. A corresponding index will be added to the indices vector.
         * \param[in] point_arg point to be added
         * \param[in] cloud_arg pointer to input point cloud dataset (given by \a setInputCloud)
         * \param[in] indices_arg pointer to indices vector of the dataset (given by \a setInputCloud)
         */
        void
        addPointToCloud (const PointT& point_arg, PointCloudPtr cloud_arg, IndicesPtr indices_arg);

        /** \brief Check if voxel at given point exist.
         * \param[in] point_arg point to be checked
         * \return "true" if voxel exist; "false" otherwise
         */
        bool
        isVoxelOccupiedAtPoint (const PointT& point_arg) const;

        /** \brief Delete the octree structure and its leaf nodes.
         * */
        void deleteTree ()
        {
          // reset bounding box
          min_x_ = min_y_ = max_y_ = min_z_ = max_z_ = 0;
          this->bounding_box_defined_ = false;

          OctreeT::deleteTree ();
        }

        /** \brief Check if voxel at given point coordinates exist.
         * \param[in] point_x_arg X coordinate of point to be checked
         * \param[in] point_y_arg Y coordinate of point to be checked
         * \param[in] point_z_arg Z coordinate of point to be checked
         * \return "true" if voxel exist; "false" otherwise
         */
        bool
        isVoxelOccupiedAtPoint (const double point_x_arg, const double point_y_arg, const double point_z_arg) const;

        /** \brief Check if voxel at given point from input cloud exist.
         * \param[in] point_idx_arg point to be checked
         * \return "true" if voxel exist; "false" otherwise
         */
        bool
        isVoxelOccupiedAtPoint (const int& point_idx_arg) const;

        /** \brief Get a PointT vector of centers of all occupied voxels.
         * \param[out] voxel_center_list_arg results are written to this vector of PointT elements
         * \return number of occupied voxels
         */
        int
        getOccupiedVoxelCenters (AlignedPointTVector &voxel_center_list_arg) const;

        /** \brief Get a PointT vector of centers of voxels intersected by a line segment.
         * This returns a approximation of the actual intersected voxels by walking
         * along the line with small steps. Voxels are ordered, from closest to
         * furthest w.r.t. the origin.
         * \param[in] origin origin of the line segment
         * \param[in] end end of the line segment
         * \param[out] voxel_center_list results are written to this vector of PointT elements
         * \param[in] precision determines the size of the steps: step_size = octree_resolution x precision
         * \return number of intersected voxels
         */
        int
        getApproxIntersectedVoxelCentersBySegment (
            const Eigen::Vector3f& origin, const Eigen::Vector3f& end,
            AlignedPointTVector &voxel_center_list, float precision = 0.2);

        /** \brief Delete leaf node / voxel at given point
         * \param[in] point_arg point addressing the voxel to be deleted.
         */
        void
        deleteVoxelAtPoint (const PointT& point_arg);

        /** \brief Delete leaf node / voxel at given point from input cloud
         *  \param[in] point_idx_arg index of point addressing the voxel to be deleted.
         */
        void
        deleteVoxelAtPoint (const int& point_idx_arg);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Bounding box methods
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Investigate dimensions of pointcloud data set and define corresponding bounding box for octree. */
        void
        defineBoundingBox ();

        /** \brief Define bounding box for octree
         * \note Bounding box cannot be changed once the octree contains elements.
         * \param[in] min_x_arg X coordinate of lower bounding box corner
         * \param[in] min_y_arg Y coordinate of lower bounding box corner
         * \param[in] min_z_arg Z coordinate of lower bounding box corner
         * \param[in] max_x_arg X coordinate of upper bounding box corner
         * \param[in] max_y_arg Y coordinate of upper bounding box corner
         * \param[in] max_z_arg Z coordinate of upper bounding box corner
         */
        void
        defineBoundingBox (const double min_x_arg, const double min_y_arg, const double min_z_arg,
                           const double max_x_arg, const double max_y_arg, const double max_z_arg);

        /** \brief Define bounding box for octree
         * \note Lower bounding box point is set to (0, 0, 0)
         * \note Bounding box cannot be changed once the octree contains elements.
         * \param[in] max_x_arg X coordinate of upper bounding box corner
         * \param[in] max_y_arg Y coordinate of upper bounding box corner
         * \param[in] max_z_arg Z coordinate of upper bounding box corner
         */
        void
        defineBoundingBox (const double max_x_arg, const double max_y_arg, const double max_z_arg);

        /** \brief Define bounding box cube for octree
         * \note Lower bounding box corner is set to (0, 0, 0)
         * \note Bounding box cannot be changed once the octree contains elements.
         * \param[in] cubeLen_arg side length of bounding box cube.
         */
        void
        defineBoundingBox (const double cubeLen_arg);

        /** \brief Get bounding box for octree
         * \note Bounding box cannot be changed once the octree contains elements.
         * \param[in] min_x_arg X coordinate of lower bounding box corner
         * \param[in] min_y_arg Y coordinate of lower bounding box corner
         * \param[in] min_z_arg Z coordinate of lower bounding box corner
         * \param[in] max_x_arg X coordinate of upper bounding box corner
         * \param[in] max_y_arg Y coordinate of upper bounding box corner
         * \param[in] max_z_arg Z coordinate of upper bounding box corner
         */
        void
        getBoundingBox (double& min_x_arg, double& min_y_arg, double& min_z_arg,
                        double& max_x_arg, double& max_y_arg, double& max_z_arg) const;

        /** \brief Calculates the squared diameter of a voxel at given tree depth
         * \param[in] tree_depth_arg depth/level in octree
         * \return squared diameter
         */
        double
        getVoxelSquaredDiameter (unsigned int tree_depth_arg) const;

        /** \brief Calculates the squared diameter of a voxel at leaf depth
         * \return squared diameter
         */
        inline double
        getVoxelSquaredDiameter () const
        {
          return getVoxelSquaredDiameter (this->octree_depth_);
        }

        /** \brief Calculates the squared voxel cube side length at given tree depth
         * \param[in] tree_depth_arg depth/level in octree
         * \return squared voxel cube side length
         */
        double
        getVoxelSquaredSideLen (unsigned int tree_depth_arg) const;

        /** \brief Calculates the squared voxel cube side length at leaf level
         * \return squared voxel cube side length
         */
        inline double getVoxelSquaredSideLen () const
        {
          return getVoxelSquaredSideLen (this->octree_depth_);
        }

        /** \brief Generate bounds of the current voxel of an octree iterator
         * \param[in] iterator: octree iterator
         * \param[out] min_pt lower bound of voxel
         * \param[out] max_pt upper bound of voxel
         */
        inline void
        getVoxelBounds (const OctreeIteratorBase<OctreeT>& iterator, Eigen::Vector3f &min_pt, Eigen::Vector3f &max_pt) const
        {
          this->genVoxelBoundsFromOctreeKey (iterator.getCurrentOctreeKey (),
              iterator.getCurrentOctreeDepth (), min_pt, max_pt);
        }

        /** \brief Enable dynamic octree structure
         *  \note Leaf nodes are kept as close to the root as possible and are only expanded if the number of DataT objects within a leaf node exceeds a fixed limit.
         *  \param maxObjsPerLeaf: maximum number of DataT objects per leaf
         * */
        inline void
        enableDynamicDepth ( size_t maxObjsPerLeaf )
        {
          assert(this->leaf_count_==0);
          max_objs_per_leaf_ = maxObjsPerLeaf;

          this->dynamic_depth_enabled_ = max_objs_per_leaf_ > 0;
        }


      protected:

        /** \brief Add point at index from input pointcloud dataset to octree
         * \param[in] point_idx_arg the index representing the point in the dataset given by \a setInputCloud to be added
         */
        virtual void
        addPointIdx (const int point_idx_arg);

        /** \brief Add point at index from input pointcloud dataset to octree
         * \param[in] leaf_node to be expanded
         * \param[in] parent_branch parent of leaf node to be expanded
         * \param[in] child_idx child index of leaf node (in parent branch)
         * \param[in] depth_mask of leaf node to be expanded
         */
        void
        expandLeafNode (LeafNode* leaf_node, BranchNode* parent_branch, unsigned char child_idx, unsigned int depth_mask);

        /** \brief Get point at index from input pointcloud dataset
         * \param[in] index_arg index representing the point in the dataset given by \a setInputCloud
         * \return PointT from input pointcloud dataset
         */
        const PointT&
        getPointByIndex (const unsigned int index_arg) const;

        /** \brief Find octree leaf node at a given point
         * \param[in] point_arg query point
         * \return pointer to leaf node. If leaf node does not exist, pointer is 0.
         */
        LeafContainerT*
        findLeafAtPoint (const PointT& point_arg) const
        {
          OctreeKey key;

          // generate key for point
          this->genOctreeKeyforPoint (point_arg, key);

          return (this->findLeaf (key));
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Protected octree methods based on octree keys
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Define octree key setting and octree depth based on defined bounding box. */
        void
        getKeyBitSize ();

        /** \brief Grow the bounding box/octree until point fits
         * \param[in] point_idx_arg point that should be within bounding box;
         */
        void
        adoptBoundingBoxToPoint (const PointT& point_idx_arg);

        /** \brief Checks if given point is within the bounding box of the octree
         * \param[in] point_idx_arg point to be checked for bounding box violations
         * \return "true" - no bound violation
         */
        inline bool isPointWithinBoundingBox (const PointT& point_idx_arg) const
        {
          return (! ( (point_idx_arg.x < min_x_) || (point_idx_arg.y < min_y_)
                   || (point_idx_arg.z < min_z_) || (point_idx_arg.x >= max_x_)
                   || (point_idx_arg.y >= max_y_) || (point_idx_arg.z >= max_z_)));
        }

        /** \brief Generate octree key for voxel at a given point
         * \param[in] point_arg the point addressing a voxel
         * \param[out] key_arg write octree key to this reference
         */
        void
        genOctreeKeyforPoint (const PointT & point_arg,
            OctreeKey &key_arg) const;

        /** \brief Generate octree key for voxel at a given point
         * \param[in] point_x_arg X coordinate of point addressing a voxel
         * \param[in] point_y_arg Y coordinate of point addressing a voxel
         * \param[in] point_z_arg Z coordinate of point addressing a voxel
         * \param[out] key_arg write octree key to this reference
         */
        void
        genOctreeKeyforPoint (const double point_x_arg, const double point_y_arg, const double point_z_arg,
                              OctreeKey & key_arg) const;

        /** \brief Virtual method for generating octree key for a given point index.
         * \note This method enables to assign indices to leaf nodes during octree deserialization.
         * \param[in] data_arg index value representing a point in the dataset given by \a setInputCloud
         * \param[out] key_arg write octree key to this reference
         * \return "true" - octree keys are assignable
         */
        virtual bool
        genOctreeKeyForDataT (const int& data_arg, OctreeKey & key_arg) const;

        /** \brief Generate a point at center of leaf node voxel
         * \param[in] key_arg octree key addressing a leaf node.
         * \param[out] point_arg write leaf node voxel center to this point reference
         */
        void
        genLeafNodeCenterFromOctreeKey (const OctreeKey & key_arg,
            PointT& point_arg) const;

        /** \brief Generate a point at center of octree voxel at given tree level
         * \param[in] key_arg octree key addressing an octree node.
         * \param[in] tree_depth_arg octree depth of query voxel
         * \param[out] point_arg write leaf node center point to this reference
         */
        void
        genVoxelCenterFromOctreeKey (const OctreeKey & key_arg,
            unsigned int tree_depth_arg, PointT& point_arg) const;

        /** \brief Generate bounds of an octree voxel using octree key and tree depth arguments
         * \param[in] key_arg octree key addressing an octree node.
         * \param[in] tree_depth_arg octree depth of query voxel
         * \param[out] min_pt lower bound of voxel
         * \param[out] max_pt upper bound of voxel
         */
        void
        genVoxelBoundsFromOctreeKey (const OctreeKey & key_arg,
            unsigned int tree_depth_arg, Eigen::Vector3f &min_pt,
            Eigen::Vector3f &max_pt) const;

        /** \brief Recursively search the tree for all leaf nodes and return a vector of voxel centers.
         * \param[in] node_arg current octree node to be explored
         * \param[in] key_arg octree key addressing a leaf node.
         * \param[out] voxel_center_list_arg results are written to this vector of PointT elements
         * \return number of voxels found
         */
        int
        getOccupiedVoxelCentersRecursive (const BranchNode* node_arg,
            const OctreeKey& key_arg,
            AlignedPointTVector &voxel_center_list_arg) const;

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Globals
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /** \brief Pointer to input point cloud dataset. */
        PointCloudConstPtr input_;

        /** \brief A pointer to the vector of point indices to use. */
        IndicesConstPtr indices_;

        /** \brief Epsilon precision (error bound) for nearest neighbors searches. */
        double epsilon_;

        /** \brief Octree resolution. */
        double resolution_;

        // Octree bounding box coordinates
        double min_x_;
        double max_x_;

        double min_y_;
        double max_y_;

        double min_z_;
        double max_z_;

        /** \brief Flag indicating if octree has defined bounding box. */
        bool bounding_box_defined_;

        /** \brief Amount of DataT objects per leafNode before expanding branch
         *  \note zero indicates a fixed/maximum depth octree structure
         * **/
        std::size_t max_objs_per_leaf_;
    };

  }
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/octree/impl/octree_pointcloud.hpp>
#endif

/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 */

#ifndef PCL_MY_OCTREE_POINTCLOUD_HPP_
#define PCL_MY_OCTREE_POINTCLOUD_HPP_

#include <cassert>

#include <pcl/common/common.h>
#include <pcl/octree/impl/octree_base.hpp>

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT>
pcl::octree::MyOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::MyOctreePointCloud (const double resolution) :
    OctreeT (), input_ (PointCloudConstPtr ()), indices_ (IndicesConstPtr ()),
    epsilon_ (0), resolution_ (resolution), min_x_ (0.0f), max_x_ (resolution), min_y_ (0.0f),
    max_y_ (resolution), min_z_ (0.0f), max_z_ (resolution), bounding_box_defined_ (false), max_objs_per_leaf_(0)
{
  assert (resolution > 0.0f);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT>
pcl::octree::MyOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::~MyOctreePointCloud ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT> void
pcl::octree::MyOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::addPointsFromInputCloud ()
{
  if (indices_)
  {
    for (const int &index : *indices_)
    {
      assert( (index >= 0) && (index < static_cast<int> (input_->points.size ())));
      
      if (isFinite (input_->points[index]))
      {
        // add points to octree
        this->addPointIdx (index);
      }
    }
  }
  else
  {
    for (size_t i = 0; i < input_->points.size (); i++)
    {
      if (isFinite (input_->points[i]))
      {
        // add points to octree
        this->addPointIdx (static_cast<unsigned int> (i));
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT> void
pcl::octree::MyOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::addPointFromCloud (const int point_idx_arg, IndicesPtr indices_arg)
{
  this->addPointIdx (point_idx_arg);
  if (indices_arg)
    indices_arg->push_back (point_idx_arg);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT> void
pcl::octree::MyOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::addPointToCloud (const PointT& point_arg, PointCloudPtr cloud_arg)
{
  assert (cloud_arg==input_);

  cloud_arg->push_back (point_arg);

  this->addPointIdx (static_cast<const int> (cloud_arg->points.size ()) - 1);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT> void
pcl::octree::MyOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::addPointToCloud (const PointT& point_arg, PointCloudPtr cloud_arg,
                                                           IndicesPtr indices_arg)
{
  assert (cloud_arg==input_);
  assert (indices_arg==indices_);

  cloud_arg->push_back (point_arg);

  this->addPointFromCloud (static_cast<const int> (cloud_arg->points.size ()) - 1, indices_arg);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT> bool
pcl::octree::MyOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::isVoxelOccupiedAtPoint (const PointT& point_arg) const
{
  if (!isPointWithinBoundingBox (point_arg))
  {
    return false;
  }

  OctreeKey key;

  // generate key for point
  this->genOctreeKeyforPoint (point_arg, key);

  // search for key in octree
  return (this->existLeaf (key));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT> bool
pcl::octree::MyOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::isVoxelOccupiedAtPoint (const int& point_idx_arg) const
{
  // retrieve point from input cloud
  const PointT& point = this->input_->points[point_idx_arg];

  // search for voxel at point in octree
  return (this->isVoxelOccupiedAtPoint (point));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT> bool
pcl::octree::MyOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::isVoxelOccupiedAtPoint (
    const double point_x_arg, const double point_y_arg, const double point_z_arg) const
{
  // create a new point with the argument coordinates
  PointT point;
  point.x = point_x_arg;
  point.y = point_y_arg;
  point.z = point_z_arg;

  // search for voxel at point in octree
  return (this->isVoxelOccupiedAtPoint (point));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT> void
pcl::octree::MyOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::deleteVoxelAtPoint (const PointT& point_arg)
{
  if (!isPointWithinBoundingBox (point_arg))
  {
    return;
  }

  OctreeKey key;

  // generate key for point
  this->genOctreeKeyforPoint (point_arg, key);

  this->removeLeaf (key);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT> void
pcl::octree::MyOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::deleteVoxelAtPoint (const int& point_idx_arg)
{
  // retrieve point from input cloud
  const PointT& point = this->input_->points[point_idx_arg];

  // delete leaf at point
  this->deleteVoxelAtPoint (point);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT> int
pcl::octree::MyOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::getOccupiedVoxelCenters (
    AlignedPointTVector &voxel_center_list_arg) const
{
  OctreeKey key;
  key.x = key.y = key.z = 0;

  voxel_center_list_arg.clear ();

  return getOccupiedVoxelCentersRecursive (this->root_node_, key, voxel_center_list_arg);

}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT> int
pcl::octree::MyOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::getApproxIntersectedVoxelCentersBySegment (
    const Eigen::Vector3f& origin,
    const Eigen::Vector3f& end,
    AlignedPointTVector &voxel_center_list,
    float precision)
{
  Eigen::Vector3f direction = end - origin;
  float norm = direction.norm ();
  direction.normalize ();

  const float step_size = static_cast<const float> (resolution_) * precision;
  // Ensure we get at least one step for the first voxel.
  const int nsteps = std::max (1, static_cast<int> (norm / step_size));

  OctreeKey prev_key;

  bool bkeyDefined = false;

  // Walk along the line segment with small steps.
  for (int i = 0; i < nsteps; ++i)
  {
    Eigen::Vector3f p = origin + (direction * step_size * static_cast<const float> (i));

    PointT octree_p;
    octree_p.x = p.x ();
    octree_p.y = p.y ();
    octree_p.z = p.z ();

    OctreeKey key;
    this->genOctreeKeyforPoint (octree_p, key);

    // Not a new key, still the same voxel.
    if ((key == prev_key) && (bkeyDefined) )
      continue;

    prev_key = key;
    bkeyDefined = true;

    PointT center;
    genLeafNodeCenterFromOctreeKey (key, center);
    voxel_center_list.push_back (center);
  }

  OctreeKey end_key;
  PointT end_p;
  end_p.x = end.x ();
  end_p.y = end.y ();
  end_p.z = end.z ();
  this->genOctreeKeyforPoint (end_p, end_key);
  if (!(end_key == prev_key))
  {
    PointT center;
    genLeafNodeCenterFromOctreeKey (end_key, center);
    voxel_center_list.push_back (center);
  }

  return (static_cast<int> (voxel_center_list.size ()));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT> void
pcl::octree::MyOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::defineBoundingBox ()
{

  double minX, minY, minZ, maxX, maxY, maxZ;

  PointT min_pt;
  PointT max_pt;

  // bounding box cannot be changed once the octree contains elements
  assert (this->leaf_count_ == 0);

  pcl::getMinMax3D (*input_, min_pt, max_pt);

  float minValue = std::numeric_limits<float>::epsilon () * 512.0f;

  minX = min_pt.x;
  minY = min_pt.y;
  minZ = min_pt.z;

  maxX = max_pt.x + minValue;
  maxY = max_pt.y + minValue;
  maxZ = max_pt.z + minValue;

  // generate bit masks for octree
  defineBoundingBox (minX, minY, minZ, maxX, maxY, maxZ);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT> void
pcl::octree::MyOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::defineBoundingBox (const double min_x_arg,
                                                                          const double min_y_arg,
                                                                          const double min_z_arg,
                                                                          const double max_x_arg,
                                                                          const double max_y_arg,
                                                                          const double max_z_arg)
{
  // bounding box cannot be changed once the octree contains elements
  assert (this->leaf_count_ == 0);

  assert (max_x_arg >= min_x_arg);
  assert (max_y_arg >= min_y_arg);
  assert (max_z_arg >= min_z_arg);

  min_x_ = min_x_arg;
  max_x_ = max_x_arg;

  min_y_ = min_y_arg;
  max_y_ = max_y_arg;

  min_z_ = min_z_arg;
  max_z_ = max_z_arg;

  min_x_ = std::min (min_x_, max_x_);
  min_y_ = std::min (min_y_, max_y_);
  min_z_ = std::min (min_z_, max_z_);

  max_x_ = std::max (min_x_, max_x_);
  max_y_ = std::max (min_y_, max_y_);
  max_z_ = std::max (min_z_, max_z_);

  // generate bit masks for octree
  getKeyBitSize ();

  bounding_box_defined_ = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT> void
pcl::octree::MyOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::defineBoundingBox (
    const double max_x_arg, const double max_y_arg, const double max_z_arg)
{
  // bounding box cannot be changed once the octree contains elements
  assert (this->leaf_count_ == 0);

  assert (max_x_arg >= 0.0f);
  assert (max_y_arg >= 0.0f);
  assert (max_z_arg >= 0.0f);

  min_x_ = 0.0f;
  max_x_ = max_x_arg;

  min_y_ = 0.0f;
  max_y_ = max_y_arg;

  min_z_ = 0.0f;
  max_z_ = max_z_arg;

  min_x_ = std::min (min_x_, max_x_);
  min_y_ = std::min (min_y_, max_y_);
  min_z_ = std::min (min_z_, max_z_);

  max_x_ = std::max (min_x_, max_x_);
  max_y_ = std::max (min_y_, max_y_);
  max_z_ = std::max (min_z_, max_z_);

  // generate bit masks for octree
  getKeyBitSize ();

  bounding_box_defined_ = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT> void
pcl::octree::MyOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::defineBoundingBox (const double cubeLen_arg)
{
  // bounding box cannot be changed once the octree contains elements
  assert (this->leaf_count_ == 0);

  assert (cubeLen_arg >= 0.0f);

  min_x_ = 0.0f;
  max_x_ = cubeLen_arg;

  min_y_ = 0.0f;
  max_y_ = cubeLen_arg;

  min_z_ = 0.0f;
  max_z_ = cubeLen_arg;

  min_x_ = std::min (min_x_, max_x_);
  min_y_ = std::min (min_y_, max_y_);
  min_z_ = std::min (min_z_, max_z_);

  max_x_ = std::max (min_x_, max_x_);
  max_y_ = std::max (min_y_, max_y_);
  max_z_ = std::max (min_z_, max_z_);

  // generate bit masks for octree
  getKeyBitSize ();

  bounding_box_defined_ = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT> void
pcl::octree::MyOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::getBoundingBox (
    double& min_x_arg, double& min_y_arg, double& min_z_arg,
    double& max_x_arg, double& max_y_arg, double& max_z_arg) const
{
  min_x_arg = min_x_;
  min_y_arg = min_y_;
  min_z_arg = min_z_;

  max_x_arg = max_x_;
  max_y_arg = max_y_;
  max_z_arg = max_z_;
}


//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT>
void
pcl::octree::MyOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::adoptBoundingBoxToPoint (const PointT& point_idx_arg)
{

  const float minValue = std::numeric_limits<float>::epsilon ();

  // increase octree size until point fits into bounding box
  while (true)
  {
    bool bLowerBoundViolationX = (point_idx_arg.x < min_x_);
    bool bLowerBoundViolationY = (point_idx_arg.y < min_y_);
    bool bLowerBoundViolationZ = (point_idx_arg.z < min_z_);

    bool bUpperBoundViolationX = (point_idx_arg.x >= max_x_);
    bool bUpperBoundViolationY = (point_idx_arg.y >= max_y_);
    bool bUpperBoundViolationZ = (point_idx_arg.z >= max_z_);

    // do we violate any bounds?
    if (bLowerBoundViolationX || bLowerBoundViolationY || bLowerBoundViolationZ || bUpperBoundViolationX
        || bUpperBoundViolationY || bUpperBoundViolationZ || (!bounding_box_defined_) )
    {

      if (bounding_box_defined_)
      {

        double octreeSideLen;
        unsigned char child_idx;

        // octree not empty - we add another tree level and thus increase its size by a factor of 2*2*2
        child_idx = static_cast<unsigned char> (((!bUpperBoundViolationX) << 2) | ((!bUpperBoundViolationY) << 1)
            | ((!bUpperBoundViolationZ)));

        BranchNode* newRootBranch;
        #ifdef BUFFER_MAX
          newRootBranch = new BranchNode();
        #else
          newRootBranch = new BranchNode(this->buffer_size_);
        #endif
        this->branch_count_++;

        this->setBranchChildPtr (*newRootBranch, child_idx, this->root_node_);

        this->root_node_ = newRootBranch;

        octreeSideLen = static_cast<double> (1 << this->octree_depth_) * resolution_;

        if (!bUpperBoundViolationX)
          min_x_ -= octreeSideLen;

        if (!bUpperBoundViolationY)
          min_y_ -= octreeSideLen;

        if (!bUpperBoundViolationZ)
          min_z_ -= octreeSideLen;

        // configure tree depth of octree
        this->octree_depth_++;
        this->setTreeDepth (this->octree_depth_);

        // recalculate bounding box width
        octreeSideLen = static_cast<double> (1 << this->octree_depth_) * resolution_ - minValue;

        // increase octree bounding box
        max_x_ = min_x_ + octreeSideLen;
        max_y_ = min_y_ + octreeSideLen;
        max_z_ = min_z_ + octreeSideLen;

      }
      // bounding box is not defined - set it to point position
      else
      {
        // octree is empty - we set the center of the bounding box to our first pixel
        this->min_x_ = point_idx_arg.x - this->resolution_ / 2;
        this->min_y_ = point_idx_arg.y - this->resolution_ / 2;
        this->min_z_ = point_idx_arg.z - this->resolution_ / 2;

        this->max_x_ = point_idx_arg.x + this->resolution_ / 2;
        this->max_y_ = point_idx_arg.y + this->resolution_ / 2;
        this->max_z_ = point_idx_arg.z + this->resolution_ / 2;

        getKeyBitSize ();

        bounding_box_defined_ = true;
      }

    }
    else
      // no bound violations anymore - leave while loop
      break;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT> void
pcl::octree::MyOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::expandLeafNode (LeafNode* leaf_node, BranchNode* parent_branch, unsigned char child_idx, unsigned int depth_mask)
{

  if (depth_mask)
  {
    // get amount of objects in leaf container
    size_t leaf_obj_count = (*leaf_node)->getSize ();

  // copy leaf data
    std::vector<int> leafIndices;
    leafIndices.reserve(leaf_obj_count);

    (*leaf_node)->getPointIndices(leafIndices);

    // delete current leaf node
    this->deleteBranchChild(*parent_branch, child_idx);
    this->leaf_count_[this->buffer_selector_] --;

    // create new branch node
    BranchNode* childBranch = this->createBranchChild (*parent_branch, child_idx);
    this->branch_count_ ++;

    // add data to new branch
    OctreeKey new_index_key;

    for (const int &leafIndex : leafIndices)
    {

      const PointT& point_from_index = input_->points[leafIndex];
      // generate key
      genOctreeKeyforPoint (point_from_index, new_index_key);

      LeafNode* newLeaf;
      BranchNode* newBranchParent;
      this->createLeafRecursive (new_index_key, depth_mask, childBranch, newLeaf, newBranchParent);

      (*newLeaf)->addPointIndex(leafIndex);
    }
  }


}


//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT> void
pcl::octree::MyOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::addPointIdx (const int point_idx_arg)
{
  OctreeKey key;
  assert (point_idx_arg < static_cast<int> (input_->points.size ()));

  const PointT& point = input_->points[point_idx_arg];

  // make sure bounding box is big enough
  adoptBoundingBoxToPoint (point);

  // generate key
  genOctreeKeyforPoint (point, key);

  LeafNode* leaf_node;
  BranchNode* parent_branch_of_leaf_node;
  unsigned int depth_mask = this->createLeafRecursive (key, this->depth_mask_ ,this->root_node_, leaf_node, parent_branch_of_leaf_node);

  if (this->dynamic_depth_enabled_ && depth_mask)
  {
    // get amount of objects in leaf container
    size_t leaf_obj_count = (*leaf_node)->getSize ();

    while  (leaf_obj_count>=max_objs_per_leaf_ && depth_mask)
    {
      // index to branch child
      unsigned char child_idx = key.getChildIdxWithDepthMask (depth_mask*2);

      expandLeafNode (leaf_node,
                      parent_branch_of_leaf_node,
                      child_idx,
                      depth_mask);

      depth_mask = this->createLeafRecursive (key, this->depth_mask_ ,this->root_node_, leaf_node, parent_branch_of_leaf_node);
      leaf_obj_count = (*leaf_node)->getSize ();
    }

  }
  
  (*leaf_node)->addPointIndex (point_idx_arg);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT> const PointT&
pcl::octree::MyOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::getPointByIndex (const unsigned int index_arg) const
{
  // retrieve point from input cloud
  assert (index_arg < static_cast<unsigned int> (input_->points.size ()));
  return (this->input_->points[index_arg]);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT> void
pcl::octree::MyOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::getKeyBitSize ()
{
  unsigned int max_voxels;

  unsigned int max_key_x;
  unsigned int max_key_y;
  unsigned int max_key_z;

  double octree_side_len;

  const float minValue = std::numeric_limits<float>::epsilon();

  // find maximum key values for x, y, z
  max_key_x = static_cast<unsigned int> (ceil ((max_x_ - min_x_ - minValue) / resolution_));
  max_key_y = static_cast<unsigned int> (ceil ((max_y_ - min_y_ - minValue) / resolution_));
  max_key_z = static_cast<unsigned int> (ceil ((max_z_ - min_z_ - minValue) / resolution_));

  // find maximum amount of keys
  max_voxels = std::max (std::max (std::max (max_key_x, max_key_y), max_key_z), static_cast<unsigned int> (2));


  // tree depth == amount of bits of max_voxels
  this->octree_depth_ = std::max ((std::min (static_cast<unsigned int> (OctreeKey::maxDepth), static_cast<unsigned int> (std::ceil (std::log2 (max_voxels) - minValue)))),
                                  static_cast<unsigned int> (0));

  octree_side_len = static_cast<double> (1 << this->octree_depth_) * resolution_;

  if (this->leaf_count_ == 0)
  {
    double octree_oversize_x;
    double octree_oversize_y;
    double octree_oversize_z;

    octree_oversize_x = (octree_side_len - (max_x_ - min_x_)) / 2.0;
    octree_oversize_y = (octree_side_len - (max_y_ - min_y_)) / 2.0;
    octree_oversize_z = (octree_side_len - (max_z_ - min_z_)) / 2.0;

    assert (octree_oversize_x > -minValue);
    assert (octree_oversize_y > -minValue);
    assert (octree_oversize_z > -minValue);

    if (octree_oversize_x > minValue)
    {
      min_x_ -= octree_oversize_x;
      max_x_ += octree_oversize_x;
    }
    if (octree_oversize_y > minValue)
    {
      min_y_ -= octree_oversize_y;
      max_y_ += octree_oversize_y;
    }
    if (octree_oversize_z > minValue)
    {
      min_z_ -= octree_oversize_z;
      max_z_ += octree_oversize_z;
    }
  }
  else
  {
    max_x_ = min_x_ + octree_side_len;
    max_y_ = min_y_ + octree_side_len;
    max_z_ = min_z_ + octree_side_len;
  }

 // configure tree depth of octree
  this->setTreeDepth (this->octree_depth_);

}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT> void
pcl::octree::MyOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::genOctreeKeyforPoint (const PointT& point_arg,
                                                                             OctreeKey & key_arg) const
  {
    // calculate integer key for point coordinates
    key_arg.x = static_cast<unsigned int> ((point_arg.x - this->min_x_) / this->resolution_);
    key_arg.y = static_cast<unsigned int> ((point_arg.y - this->min_y_) / this->resolution_);
    key_arg.z = static_cast<unsigned int> ((point_arg.z - this->min_z_) / this->resolution_);
    
    assert (key_arg.x <= this->max_key_.x);
    assert (key_arg.y <= this->max_key_.y);
    assert (key_arg.z <= this->max_key_.z);
  }

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT> void
pcl::octree::MyOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::genOctreeKeyforPoint (
    const double point_x_arg, const double point_y_arg,
    const double point_z_arg, OctreeKey & key_arg) const
{
  PointT temp_point;

  temp_point.x = static_cast<float> (point_x_arg);
  temp_point.y = static_cast<float> (point_y_arg);
  temp_point.z = static_cast<float> (point_z_arg);

  // generate key for point
  genOctreeKeyforPoint (temp_point, key_arg);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT> bool
pcl::octree::MyOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::genOctreeKeyForDataT (const int& data_arg, OctreeKey & key_arg) const
{
  const PointT temp_point = getPointByIndex (data_arg);

  // generate key for point
  genOctreeKeyforPoint (temp_point, key_arg);

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT> void
pcl::octree::MyOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::genLeafNodeCenterFromOctreeKey (const OctreeKey & key, PointT & point) const
{
  // define point to leaf node voxel center
  point.x = static_cast<float> ((static_cast<double> (key.x) + 0.5f) * this->resolution_ + this->min_x_);
  point.y = static_cast<float> ((static_cast<double> (key.y) + 0.5f) * this->resolution_ + this->min_y_);
  point.z = static_cast<float> ((static_cast<double> (key.z) + 0.5f) * this->resolution_ + this->min_z_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT> void
pcl::octree::MyOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::genVoxelCenterFromOctreeKey (
    const OctreeKey & key_arg,
    unsigned int tree_depth_arg,
    PointT& point_arg) const
{
  // generate point for voxel center defined by treedepth (bitLen) and key
  point_arg.x = static_cast<float> ((static_cast <double> (key_arg.x) + 0.5f) * (this->resolution_ * static_cast<double> (1 << (this->octree_depth_ - tree_depth_arg))) + this->min_x_);
  point_arg.y = static_cast<float> ((static_cast <double> (key_arg.y) + 0.5f) * (this->resolution_ * static_cast<double> (1 << (this->octree_depth_ - tree_depth_arg))) + this->min_y_);
  point_arg.z = static_cast<float> ((static_cast <double> (key_arg.z) + 0.5f) * (this->resolution_ * static_cast<double> (1 << (this->octree_depth_ - tree_depth_arg))) + this->min_z_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT> void
pcl::octree::MyOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::genVoxelBoundsFromOctreeKey (
    const OctreeKey & key_arg,
    unsigned int tree_depth_arg,
    Eigen::Vector3f &min_pt,
    Eigen::Vector3f &max_pt) const
{
  // calculate voxel size of current tree depth
  double voxel_side_len = this->resolution_ * static_cast<double> (1 << (this->octree_depth_ - tree_depth_arg));

  // calculate voxel bounds
  min_pt (0) = static_cast<float> (static_cast<double> (key_arg.x) * voxel_side_len + this->min_x_);
  min_pt (1) = static_cast<float> (static_cast<double> (key_arg.y) * voxel_side_len + this->min_y_);
  min_pt (2) = static_cast<float> (static_cast<double> (key_arg.z) * voxel_side_len + this->min_z_);

  max_pt (0) = static_cast<float> (static_cast<double> (key_arg.x + 1) * voxel_side_len + this->min_x_);
  max_pt (1) = static_cast<float> (static_cast<double> (key_arg.y + 1) * voxel_side_len + this->min_y_);
  max_pt (2) = static_cast<float> (static_cast<double> (key_arg.z + 1) * voxel_side_len + this->min_z_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT> double
pcl::octree::MyOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::getVoxelSquaredSideLen (unsigned int tree_depth_arg) const
{
  double side_len;

  // side length of the voxel cube increases exponentially with the octree depth
  side_len = this->resolution_ * static_cast<double>(1 << (this->octree_depth_ - tree_depth_arg));

  // squared voxel side length
  side_len *= side_len;

  return (side_len);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT> double
pcl::octree::MyOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::getVoxelSquaredDiameter (unsigned int tree_depth_arg) const
{
  // return the squared side length of the voxel cube as a function of the octree depth
  return (getVoxelSquaredSideLen (tree_depth_arg) * 3);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT> int
pcl::octree::MyOctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::getOccupiedVoxelCentersRecursive (
    const BranchNode* node_arg,
    const OctreeKey& key_arg,
    AlignedPointTVector &voxel_center_list_arg) const
{
  int voxel_count = 0;

  // iterate over all children
  for (unsigned char child_idx = 0; child_idx < 8; child_idx++)
  {
    if (!this->branchHasChild (*node_arg, child_idx))
      continue;

    const OctreeNode * child_node;
    child_node = this->getBranchChildPtr (*node_arg, child_idx);

    // generate new key for current branch voxel
    OctreeKey new_key;
    new_key.x = (key_arg.x << 1) | (!!(child_idx & (1 << 2)));
    new_key.y = (key_arg.y << 1) | (!!(child_idx & (1 << 1)));
    new_key.z = (key_arg.z << 1) | (!!(child_idx & (1 << 0)));

    switch (child_node->getNodeType ())
    {
      case BRANCH_NODE:
      {
        // recursively proceed with indexed child branch
        voxel_count += getOccupiedVoxelCentersRecursive (static_cast<const BranchNode*> (child_node), new_key, voxel_center_list_arg);
        break;
      }
      case LEAF_NODE:
      {
        PointT new_point;

        genLeafNodeCenterFromOctreeKey (new_key, new_point);
        voxel_center_list_arg.push_back (new_point);

        voxel_count++;
        break;
      }
      default:
        break;
    }
  }
  return (voxel_count);
}

#define PCL_INSTANTIATE_MyOctreePointCloudSingleBufferWithLeafDataTVector(T) template class PCL_EXPORTS pcl::octree::MyOctreePointCloud<T, pcl::octree::OctreeContainerPointIndices, pcl::octree::OctreeContainerEmpty, pcl::octree::OctreeBase<pcl::octree::OctreeContainerPointIndices, pcl::octree::OctreeContainerEmpty > >;
#define PCL_INSTANTIATE_MyOctreePointCloudDoubleBufferWithLeafDataTVector(T) template class PCL_EXPORTS pcl::octree::MyOctreePointCloud<T, pcl::octree::OctreeContainerPointIndices, pcl::octree::OctreeContainerEmpty, pcl::octree::Octree2BufBase<pcl::octree::OctreeContainerPointIndices, pcl::octree::OctreeContainerEmpty > >;

#define PCL_INSTANTIATE_MyOctreePointCloudSingleBufferWithLeafDataT(T) template class PCL_EXPORTS pcl::octree::MyOctreePointCloud<T, pcl::octree::OctreeContainerPointIndex, pcl::octree::OctreeContainerEmpty, pcl::octree::OctreeBase<pcl::octree::OctreeContainerPointIndex, pcl::octree::OctreeContainerEmpty > >;
#define PCL_INSTANTIATE_MyOctreePointCloudDoubleBufferWithLeafDataT(T) template class PCL_EXPORTS pcl::octree::MyOctreePointCloud<T, pcl::octree::OctreeContainerPointIndex, pcl::octree::OctreeContainerEmpty, pcl::octree::Octree2BufBase<pcl::octree::OctreeContainerPointIndex, pcl::octree::OctreeContainerEmpty > >;

#define PCL_INSTANTIATE_MyOctreePointCloudSingleBufferWithEmptyLeaf(T) template class PCL_EXPORTS pcl::octree::MyOctreePointCloud<T, pcl::octree::OctreeContainerEmpty, pcl::octree::OctreeContainerEmpty, pcl::octree::OctreeBase<pcl::octree::OctreeContainerEmpty, pcl::octree::OctreeContainerEmpty > >;
#define PCL_INSTANTIATE_MyOctreePointCloudDoubleBufferWithEmptyLeaf(T) template class PCL_EXPORTS pcl::octree::MyOctreePointCloud<T, pcl::octree::OctreeContainerEmpty, pcl::octree::OctreeContainerEmpty, pcl::octree::Octree2BufBase<pcl::octree::OctreeContainerEmpty, pcl::octree::OctreeContainerEmpty > >;

#endif /* OCTREE_POINTCLOUD_HPP_ */
