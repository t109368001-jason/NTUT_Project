#pragma once

#include <vector>

#include <pcl/octree/octree_nodes.h>
#include <pcl/octree/octree_container.h>
#include <pcl/octree/octree_key.h>
#include <pcl/octree/octree_iterator.h>

namespace pcl
{
  namespace octree
  {

    template<typename ContainerT>
    class NBufferedBranchNode : public OctreeNode
    {

      public:
        /** \brief Empty constructor. */
        #ifdef BUFFER_MAX
        NBufferedBranchNode () : OctreeNode()
        {
          reset ();
        }
        #else
        NBufferedBranchNode (unsigned short buffer_size) : OctreeNode()
        {
          child_node_array_.resize(buffer_size);
          reset ();
        }
        #endif

        /** \brief Copy constructor. */
        NBufferedBranchNode (const NBufferedBranchNode& source) : OctreeNode()
        {
          *this = source;
        }

        /** \brief Copy operator. */
        inline NBufferedBranchNode&
        operator = (const NBufferedBranchNode &source_arg)
        {
          reset();
          
          for (unsigned short b = 0; b < BUFFER_MAX; ++b)
            for (unsigned char i = 0; i < 8; ++i)
              if (source_arg.child_node_array_[b][i])
                child_node_array_[b][i] = source_arg.child_node_array_[b][i]->deepCopy ();

          return (*this);
        }

        /** \brief Empty constructor. */
        ~NBufferedBranchNode ()
        {
          for (unsigned short b = 0; b < BUFFER_MAX; ++b)
            for (unsigned char i = 0; i < 8; ++i)
              if (child_node_array_[b][i])
                delete child_node_array_[b][i];
        }

        /** \brief Method to perform a deep copy of the octree */
        virtual NBufferedBranchNode*
        deepCopy () const
        {
          return new NBufferedBranchNode (*this);
        }

        /** \brief Get child pointer in current branch node
         *  \param buffer_arg: buffer selector
         *  \param index_arg: index of child in node
         *  \return pointer to child node
         * */
        inline OctreeNode*
        getChildPtr (unsigned short buffer_arg, unsigned char index_arg) const
        {
          assert( (buffer_arg<BUFFER_MAX) && (index_arg<8));
          return child_node_array_[buffer_arg][index_arg];
        }

        /** \brief Set child pointer in current branch node
         *  \param buffer_arg: buffer selector
         *  \param index_arg: index of child in node
         *  \param newNode_arg: pointer to new child node
         * */
        inline void setChildPtr (unsigned short buffer_arg, unsigned char index_arg,
            OctreeNode* newNode_arg)
        {
          assert( (buffer_arg<BUFFER_MAX) && (index_arg<8));
          child_node_array_[buffer_arg][index_arg] = newNode_arg;
        }

        /** \brief Check if branch is pointing to a particular child node
         *  \param buffer_arg: buffer selector
         *  \param index_arg: index of child in node
         *  \return "true" if pointer to child node exists; "false" otherwise
         * */
        inline bool hasChild (unsigned short buffer_arg, unsigned char index_arg) const
        {
          assert( (buffer_arg<BUFFER_MAX) && (index_arg<8));
          return (child_node_array_[buffer_arg][index_arg] != 0);
        }

        /** \brief Get the type of octree node. Returns LEAVE_NODE type */
        virtual node_type_t getNodeType () const
        {
          return BRANCH_NODE;
        }

        /** \brief Reset branch node container for every branch buffer. */
        inline void reset ()
        {
          for (unsigned short b = 0; b < BUFFER_MAX; ++b)
            for (unsigned char i = 0; i < 8; ++i)
              child_node_array_[b][i] = nullptr;
        }

        /** \brief Get const pointer to container */
        const ContainerT*
        operator->() const
        {
          return &container_;
        }

        /** \brief Get pointer to container */
        ContainerT*
        operator-> ()
        {
          return &container_;
        }

        /** \brief Get const reference to container */
        const ContainerT&
        operator* () const
        {
          return container_;
        }

        /** \brief Get reference to container */
        ContainerT&
        operator* ()
        {
          return container_;
        }

        /** \brief Get const reference to container */
        const ContainerT&
        getContainer () const
        {
          return container_;
        }

        /** \brief Get reference to container */
        ContainerT&
        getContainer ()
        {
          return container_;
        }

        /** \brief Get const pointer to container */
        const ContainerT*
        getContainerPtr() const
        {
          return &container_;
        }

        /** \brief Get pointer to container */
        ContainerT*
        getContainerPtr ()
        {
          return &container_;
        }

      protected:
        ContainerT container_;
        #ifdef BUFFER_MAX
        
        OctreeNode* child_node_array_[BUFFER_MAX][8];
        //std::vector<OctreeNode*[8]> child_node_array_;
        #else
        std::vector<OctreeNode*[8]> child_node_array_;
        #endif
    };

    template<typename LeafContainerT = int,
             typename BranchContainerT = OctreeContainerEmpty >
    class OctreeNBufBase
    {

      public:

        typedef OctreeNBufBase<LeafContainerT, BranchContainerT> OctreeT;

        // iterators are friends
        //friend class OctreeIteratorBase<OctreeT> ;
        //friend class OctreeDepthFirstIterator<OctreeT> ;
        //friend class OctreeBreadthFirstIterator<OctreeT> ;
        //friend class OctreeLeafNodeDepthFirstIterator<OctreeT> ;
        //friend class OctreeLeafNodeBreadthFirstIterator<OctreeT> ;

        typedef NBufferedBranchNode<BranchContainerT> BranchNode;
        typedef OctreeLeafNode<LeafContainerT> LeafNode;

        typedef BranchContainerT BranchContainer;
        typedef LeafContainerT LeafContainer;

        // Octree default iterators
        typedef OctreeDepthFirstIterator<OctreeT> Iterator;
        typedef const OctreeDepthFirstIterator<OctreeT> ConstIterator;
        Iterator begin(unsigned int max_depth_arg = 0) {return Iterator(this, max_depth_arg);};
        const Iterator end() {return Iterator();};

        /*/ Octree leaf node iterators
        // The previous deprecated names
        // LeafNodeIterator and ConstLeafNodeIterator are deprecated.
        // Please use LeafNodeDepthFirstIterator and ConstLeafNodeDepthFirstIterator instead.
        typedef OctreeLeafNodeDepthFirstIterator<OctreeT> LeafNodeIterator;
        typedef const OctreeLeafNodeDepthFirstIterator<OctreeT> ConstLeafNodeIterator;

        [[deprecated("use leaf_depth_begin() instead")]]
        LeafNodeIterator leaf_begin (unsigned int max_depth_arg = 0)
        {
          return LeafNodeIterator (this, max_depth_arg);
        };

        [[deprecated("use leaf_depth_end() instead")]]
        const LeafNodeIterator leaf_end ()
        {
          return LeafNodeIterator ();
        };
        // The currently valide names
        typedef OctreeLeafNodeDepthFirstIterator<OctreeT> LeafNodeDepthFirstIterator;
        typedef const OctreeLeafNodeDepthFirstIterator<OctreeT> ConstLeafNodeDepthFirstIterator;
        LeafNodeDepthFirstIterator leaf_depth_begin (unsigned int max_depth_arg = 0)
        {
          return LeafNodeDepthFirstIterator (this, max_depth_arg);
        };

        const LeafNodeDepthFirstIterator leaf_depth_end ()
        {
          return LeafNodeDepthFirstIterator();
        };
        //*/

        // Octree depth-first iterators
        typedef OctreeDepthFirstIterator<OctreeT> DepthFirstIterator;
        typedef const OctreeDepthFirstIterator<OctreeT> ConstDepthFirstIterator;
        DepthFirstIterator depth_begin(unsigned int maxDepth_arg = 0) {return DepthFirstIterator(this, maxDepth_arg);};
        const DepthFirstIterator depth_end() {return DepthFirstIterator();};

        // Octree breadth-first iterators
        typedef OctreeBreadthFirstIterator<OctreeT> BreadthFirstIterator;
        typedef const OctreeBreadthFirstIterator<OctreeT> ConstBreadthFirstIterator;
        BreadthFirstIterator breadth_begin(unsigned int max_depth_arg = 0) {return BreadthFirstIterator(this, max_depth_arg);};
        const BreadthFirstIterator breadth_end() {return BreadthFirstIterator();};

        /*/ Octree leaf node iterators
        typedef OctreeLeafNodeBreadthFirstIterator<OctreeT> LeafNodeBreadthIterator;
        typedef const OctreeLeafNodeBreadthFirstIterator<OctreeT> ConstLeafNodeBreadthIterator;

        LeafNodeBreadthIterator leaf_breadth_begin (unsigned int max_depth_arg = 0u)
        {
          return LeafNodeBreadthIterator (this, max_depth_arg? max_depth_arg : this->octree_depth_);
        };

        const LeafNodeBreadthIterator leaf_breadth_end ()
        {
          return LeafNodeBreadthIterator (this, 0, nullptr);
        };
        //*/

        /** \brief Empty constructor. */
        OctreeNBufBase ();

        /** \brief Empty deconstructor. */
        virtual
        ~OctreeNBufBase ();

        /** \brief Copy constructor. */
        OctreeNBufBase (const OctreeNBufBase& source) :
            leaf_count_(source.leaf_count_),
            leaf_count_all_ (source.leaf_count_all_),
            branch_count_ (source.branch_count_),
            root_node_ (new (BranchNode) (*(source.root_node_))),
            depth_mask_ (source.depth_mask_),
            max_key_ (source.max_key_),
            buffer_selector_ (source.buffer_selector_),
            octree_depth_ (source.octree_depth_),
            dynamic_depth_enabled_(source.dynamic_depth_enabled_)
        {
        }

        /** \brief Copy constructor. */
        inline OctreeNBufBase&
        operator = (const OctreeNBufBase& source)
        {
          leaf_count_ = source.leaf_count_;
          leaf_count_all_ = source.leaf_count_all_;
          branch_count_ = source.branch_count_;
          root_node_ = new (BranchNode) (* (source.root_node_));
          depth_mask_ = source.depth_mask_;
          max_key_ = source.max_key_;
          buffer_selector_ = source.buffer_selector_;
          octree_depth_ = source.octree_depth_;
          dynamic_depth_enabled_ = source.dynamic_depth_enabled_;
          return (*this);
        }

        /** \brief Set the maximum amount of voxels per dimension.
         *  \param max_voxel_index_arg: maximum amount of voxels per dimension
         * */
        void
        setMaxVoxelIndex (unsigned int max_voxel_index_arg);

        /** \brief Set the maximum depth of the octree.
         *  \param depth_arg: maximum depth of octree
         * */
        void
        setTreeDepth (unsigned int depth_arg);

        /** \brief Get the maximum depth of the octree.
         *  \return depth_arg: maximum depth of octree
         * */
        inline unsigned int getTreeDepth () const
        {
          return this->octree_depth_;
        }

        /** \brief Create new leaf node at (idx_x_arg, idx_y_arg, idx_z_arg).
         *  \note If leaf node already exist, this method returns the existing node
         *  \param idx_x_arg: index of leaf node in the X axis.
         *  \param idx_y_arg: index of leaf node in the Y axis.
         *  \param idx_z_arg: index of leaf node in the Z axis.
         *  \return pointer to new leaf node container.
         * */
        LeafContainerT*
        createLeaf (unsigned int idx_x_arg, unsigned int idx_y_arg, unsigned int idx_z_arg);

        /** \brief Find leaf node at (idx_x_arg, idx_y_arg, idx_z_arg).
         *  \note If leaf node already exist, this method returns the existing node
         *  \param idx_x_arg: index of leaf node in the X axis.
         *  \param idx_y_arg: index of leaf node in the Y axis.
         *  \param idx_z_arg: index of leaf node in the Z axis.
         *  \return pointer to leaf node container if found, null pointer otherwise.
         * */
        LeafContainerT*
        findLeaf (unsigned int idx_x_arg, unsigned int idx_y_arg, unsigned int idx_z_arg);

        /** \brief Check for the existence of leaf node at (idx_x_arg, idx_y_arg, idx_z_arg).
         *  \param idx_x_arg: index of leaf node in the X axis.
         *  \param idx_y_arg: index of leaf node in the Y axis.
         *  \param idx_z_arg: index of leaf node in the Z axis.
         *  \return "true" if leaf node search is successful, otherwise it returns "false".
         * */
        bool
        existLeaf (unsigned int idx_x_arg, unsigned int idx_y_arg, unsigned int idx_z_arg) const;

        /** \brief Remove leaf node at (idx_x_arg, idx_y_arg, idx_z_arg).
         *  \param idx_x_arg: index of leaf node in the X axis.
         *  \param idx_y_arg: index of leaf node in the Y axis.
         *  \param idx_z_arg: index of leaf node in the Z axis.
         * */
        void
        removeLeaf (unsigned int idx_x_arg, unsigned int idx_y_arg, unsigned int idx_z_arg);

        inline unsigned short getBufferSelector () const
        {
          return buffer_selector_;
        }

        /** \brief Return the amount of existing leafs in the octree.
         *  \return amount of registered leaf nodes.
         * */
        inline std::size_t getLeafCount () const
        {
          return (leaf_count_all_[getBufferSelector()]);
        }

        inline std::size_t getLeafCount (unsigned short buffer_selector_arg) const
        {
          return (leaf_count_all_[buffer_selector_arg]);
        }

        /** \brief Return the amount of existing branches in the octree.
         *  \return amount of branch nodes.
         * */
        inline std::size_t getBranchCount () const
        {
          return (branch_count_);
        }

        /** \brief Delete the octree structure and its leaf nodes.
         * */
        void
        deleteTree ();

        /** \brief Delete octree structure of previous buffer. */
        inline void deletePreviousBuffer ()
        {
          treeCleanUpRecursive (root_node_);
        }

        /** \brief Delete the octree structure in the current buffer. */
        inline void deleteCurrentBuffer ()
        {
          treeCleanUpRecursive (root_node_);
          leaf_count_all_[buffer_selector_] = 0;
        }

        /** \brief Switch buffers and reset current octree structure. */
        void
        switchBuffers (unsigned short buffer_selector_arg);

        /** \brief Switch buffers and reset current octree structure. */
        void
        nextBuffer () {
          #ifdef BUFFER_MAX
          switchBuffers((buffer_selector_ + 1 < BUFFER_MAX ? buffer_selector_ + 1 : 0));
          #else
          switchBuffers((buffer_selector_ + 1 < buffer_size_ ? buffer_selector_ + 1 : 0));
          #endif
        }

        void expendBufferRecursive (BranchNode* branch_arg, unsigned short buffer_selector_arg);

        /** \brief Serialize octree into a binary output vector describing its branch node structure.
         *  \param binary_tree_out_arg: reference to output vector for writing binary tree structure.
         *  \param do_XOR_encoding_arg: select if binary tree structure should be generated based on current octree (false) of based on a XOR comparison between current and previous octree
         * */
        void
        serializeTree (std::vector<char>& binary_tree_out_arg);

        /** \brief Serialize octree into a binary output vector describing its branch node structure and and push all DataT elements stored in the octree to a vector.
         * \param binary_tree_out_arg: reference to output vector for writing binary tree structure.
         * \param leaf_container_vector_arg: pointer to all LeafContainerT objects in the octree
         * \param do_XOR_encoding_arg: select if binary tree structure should be generated based on current octree (false) of based on a XOR comparison between current and previous octree
         * */
        void
        serializeTree (std::vector<char>& binary_tree_out_arg,
                       std::vector<LeafContainerT*>& leaf_container_vector_arg);

        /** \brief Outputs a vector of all DataT elements that are stored within the octree leaf nodes.
         *  \param leaf_container_vector_arg: vector of pointers to all LeafContainerT objects in the octree
         * */
        void
        serializeLeafs (std::vector<LeafContainerT*>& leaf_container_vector_arg);

        /** \brief Outputs a vector of all DataT elements from leaf nodes, that do not exist in the previous octree buffer.
         *  \param leaf_container_vector_arg: vector of pointers to all LeafContainerT objects in the octree
         * */
        void
        serializeNewLeafs (std::vector<LeafContainerT*>& leaf_container_vector_arg);

        /** \brief Deserialize a binary octree description vector and create a corresponding octree structure. Leaf nodes are initialized with getDataTByKey(..).
         *  \param binary_tree_in_arg: reference to input vector for reading binary tree structure.
         *  \param do_XOR_decoding_arg: select if binary tree structure is based on current octree (false) of based on a XOR comparison between current and previous octree
         * */
        void
        deserializeTree (std::vector<char>& binary_tree_in_arg);

        /** \brief Deserialize a binary octree description and create a corresponding octree structure. Leaf nodes are initialized with DataT elements from the dataVector.
         *  \param binary_tree_in_arg: reference to inpvectoream for reading binary tree structure.
         *  \param leaf_container_vector_arg: vector of pointers to all LeafContainerT objects in the octree
         *  \param do_XOR_decoding_arg: select if binary tree structure is based on current octree (false) of based on a XOR comparison between current and previous octree
         * */
        void
        deserializeTree (std::vector<char>& binary_tree_in_arg,
                         std::vector<LeafContainerT*>& leaf_container_vector_arg);

        void
        serializeLeafs (std::vector<LeafContainerT*>& leaf_container_vector_arg,
                        unsigned short buffer_selector_arg,
                        boost::function<bool ( std::vector<bool> bit_patterns )>* function = nullptr,
                        std::vector<std::pair<unsigned char, LeafContainerT*>>* all_buffer_vector_arg = nullptr,
                        bool output_multi_buf_data = false);

      protected:

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Protected octree methods based on octree keys
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Retrieve root node */
        OctreeNode*
        getRootNode () const
        {
          return (this->root_node_);
        }

        /** \brief Find leaf node
         *  \param key_arg: octree key addressing a leaf node.
         *  \return pointer to leaf container. If leaf node is not found, this pointer returns 0.
         * */
        inline LeafContainerT*
        findLeaf (const OctreeKey& key_arg) const
        {
          LeafContainerT* result = nullptr;
          findLeafRecursive (key_arg, depth_mask_, root_node_, result);
          return result;
        }

        /** \brief Create a leaf node.
         *  \note If the leaf node at the given octree node does not exist, it will be created and added to the tree.
         *  \param key_arg: octree key addressing a leaf node.
         *  \return pointer to an existing or created leaf container.
         * */
        inline LeafContainerT*
        createLeaf (const OctreeKey& key_arg)
        {
          LeafNode* leaf_node;
          BranchNode* leaf_node_parent;

          createLeafRecursive (key_arg, depth_mask_ ,root_node_, leaf_node, leaf_node_parent, false);

          LeafContainerT* ret = leaf_node->getContainerPtr();

          return ret;
        }

        /** \brief Check if leaf doesn't exist in the octree
         *  \param key_arg: octree key addressing a leaf node.
         *  \return "true" if leaf node is found; "false" otherwise
         * */
        inline bool existLeaf (const OctreeKey& key_arg) const
        {
          return (findLeaf(key_arg) != nullptr);
        }

        /** \brief Remove leaf node from octree
         *  \param key_arg: octree key addressing a leaf node.
         * */
        inline void removeLeaf (const OctreeKey& key_arg)
        {
          if (key_arg <= max_key_)
          {
            deleteLeafRecursive (key_arg, depth_mask_, root_node_);
          }
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Branch node accessor inline functions
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Check if branch is pointing to a particular child node
         *  \param branch_arg: reference to octree branch class
         *  \param child_idx_arg: index to child node
         *  \return "true" if pointer to child node exists; "false" otherwise
         * */
        inline bool
        branchHasChild (const BranchNode& branch_arg, unsigned char child_idx_arg) const
        {
          // test occupancyByte for child existence
          return (branch_arg.getChildPtr(buffer_selector_, child_idx_arg) != nullptr);
        }

        /** \brief Retrieve a child node pointer for child node at child_idx.
         * \param branch_arg: reference to octree branch class
         * \param child_idx_arg: index to child node
         * \return pointer to octree child node class
         */
        inline OctreeNode*
        getBranchChildPtr (const BranchNode& branch_arg,
            unsigned char child_idx_arg) const
        {
          return branch_arg.getChildPtr(buffer_selector_, child_idx_arg);
        }

        /** \brief Assign new child node to branch
         *  \param branch_arg: reference to octree branch class
         *  \param child_idx_arg: index to child node
         *  \param new_child_arg: pointer to new child node
         * */
        inline void
        setBranchChildPtr (BranchNode& branch_arg, unsigned char child_idx_arg, OctreeNode* new_child_arg)
        {
          branch_arg.setChildPtr (buffer_selector_, child_idx_arg, new_child_arg);
        }

        /** \brief Generate bit pattern reflecting the existence of child node pointers for current buffer
         *  \param branch_arg: reference to octree branch class
         *  \return a single byte with 8 bits of child node information
         * */
        inline char getBranchBitPattern (const BranchNode& branch_arg) const
        {
          char node_bits;

          // create bit pattern
          node_bits = 0;
          for (unsigned char i = 0; i < 8; i++)
          {
            const OctreeNode* child = branch_arg.getChildPtr(buffer_selector_, i);
            node_bits |= static_cast<char> ( (!!child) << i);
          }

          return (node_bits);
        }

        /** \brief Generate bit pattern reflecting the existence of child node pointers in specific buffer
         *  \param branch_arg: reference to octree branch class
         *  \param buffer_selector_arg: buffer selector
         *  \return a single byte with 8 bits of child node information
         * */
        inline char getBranchBitPattern (const BranchNode& branch_arg,
            unsigned short buffer_selector_arg) const
        {
          char node_bits;

          // create bit pattern
          node_bits = 0;
          for (unsigned char i = 0; i < 8; i++)
          {
            const OctreeNode* child = branch_arg.getChildPtr(buffer_selector_arg, i);
            node_bits |= static_cast<char> ( (!!child) << i);
          }

          return (node_bits);
        }

        inline char getBranchUniqueHasBitPattern (const BranchNode& branch_arg,
            unsigned short buffer_selector_arg) const
        {
          char unique_has_bits;

          // create bit pattern
          unique_has_bits = getBranchBitPattern(branch_arg, buffer_selector_arg);
          #ifdef BUFFER_MAX
          for(unsigned short selector = 0; selector < BUFFER_MAX; selector++)
          #else
          for(unsigned short selector = 0; selector < buffer_size_; selector++)
          #endif
          {
            if(selector == buffer_selector_arg) continue;
            char node_bits = getBranchBitPattern(branch_arg, selector);
            unique_has_bits &= (!node_bits);
          }
          
          return (unique_has_bits);
        }

        /** \brief Generate XOR bit pattern reflecting differences between the two octree buffers
         *  \param branch_arg: reference to octree branch class
         *  \return a single byte with 8 bits of child node XOR difference information
         * *
        inline char getBranchXORBitPattern (
            const BranchNode& branch_arg,
            unsigned short buffer_selector_arg) const
        {
          char node_bits[2];

          // create bit pattern for both buffers
          node_bits[0] = node_bits[1] = 0;

          for (unsigned char i = 0; i < 8; i++)
          {
            const OctreeNode* childA = branch_arg.getChildPtr(buffer_selector_, i);
            const OctreeNode* childB = branch_arg.getChildPtr(buffer_selector_arg, i);

            node_bits[0] |= static_cast<char> ( (!!childA) << i);
            node_bits[1] |= static_cast<char> ( (!!childB) << i);
          }

          return node_bits[0] ^ node_bits[1];
        }
        //*/

        /** \brief Test if branch changed between previous and current buffer
         *  \param branch_arg: reference to octree branch class
         *  \return "true", if child node information differs between current and previous octree buffer
         * */
        inline bool hasBranchChanges (const BranchNode& branch_arg) const
        {
          #ifdef BUFFER_MAX
          for(unsigned short selector = 0; selector < BUFFER_MAX; selector++)
          #else
          for(unsigned short selector = 0; selector < buffer_size_; selector++)
          #endif
          {
            if(getBranchXORBitPattern (branch_arg, selector) > 0)
            {
              return true;
            }
          }
          return false;
        }

        /** \brief Delete child node and all its subchilds from octree in specific buffer
         *  \param branch_arg: reference to octree branch class
         *  \param buffer_selector_arg: buffer selector
         *  \param child_idx_arg: index to child node
         * */
        inline void deleteBranchChild (BranchNode& branch_arg,
            unsigned short buffer_selector_arg,
            unsigned char child_idx_arg)
        {
          if (branch_arg.hasChild(buffer_selector_arg, child_idx_arg))
          {
            OctreeNode* branchChild = branch_arg.getChildPtr(buffer_selector_arg, child_idx_arg);

            switch (branchChild->getNodeType ())
            {
              case BRANCH_NODE:
              {
                // free child branch recursively
                deleteBranchChild (*static_cast<BranchNode*> (branchChild),
                                   buffer_selector_arg,
                                   child_idx_arg);

                // delete unused branch
                delete (static_cast<BranchNode*> (branchChild));
                break;
              }

              case LEAF_NODE:
              {
                // push unused leaf to branch pool
                delete (static_cast<LeafNode*> (branchChild));
                break;
              }
              default:
                break;
            }

            // set branch child pointer to 0
            branch_arg.setChildPtr(buffer_selector_arg, child_idx_arg, nullptr);
          }
        }

        /** \brief Delete child node and all its subchilds from octree in current buffer
         *  \param branch_arg: reference to octree branch class
         *  \param child_idx_arg: index to child node
         * */
        inline void deleteBranchChild (BranchNode& branch_arg,  unsigned char child_idx_arg)
        {
          deleteBranchChild(branch_arg, buffer_selector_, child_idx_arg);
        }

        /** \brief Delete branch and all its subchilds from octree (both buffers)
         *  \param branch_arg: reference to octree branch class
         * */
        inline void deleteBranch (BranchNode& branch_arg)
        {
          // delete all branch node children
          for (char i = 0; i < 8; i++)
          {
            #ifdef BUFFER_MAX
            for(unsigned short selector = 0; selector < BUFFER_MAX; selector++)
            #else
            for(unsigned short selector = 0; selector < buffer_size_; selector++)
            #endif
            {
              // remove pointers from both buffers
              deleteBranchChild (branch_arg, selector, i);
              branch_arg.setChildPtr(selector, i, nullptr);
            }
          }
        }

        /** \brief Fetch and add a new branch child to a branch class in current buffer
         *  \param branch_arg: reference to octree branch class
         *  \param child_idx_arg: index to child node
         *  \return pointer of new branch child to this reference
         * */
        inline  BranchNode* createBranchChild (BranchNode& branch_arg,
            unsigned char child_idx_arg)
        {
          BranchNode* new_branch_child = new BranchNode();

          branch_arg.setChildPtr (buffer_selector_, child_idx_arg,
              static_cast<OctreeNode*> (new_branch_child));

          return new_branch_child;
        }

        /** \brief Fetch and add a new leaf child to a branch class
         *  \param branch_arg: reference to octree branch class
         *  \param child_idx_arg: index to child node
         *  \return pointer of new leaf child to this reference
         * */
        inline LeafNode*
        createLeafChild (BranchNode& branch_arg, unsigned char child_idx_arg)
        {
          LeafNode* new_leaf_child = new LeafNode();

          branch_arg.setChildPtr(buffer_selector_, child_idx_arg, new_leaf_child);

          return new_leaf_child;
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Recursive octree methods
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Create a leaf node at octree key. If leaf node does already exist, it is returned.
         *  \param key_arg: reference to an octree key
         *  \param depth_mask_arg: depth mask used for octree key analysis and for branch depth indicator
         *  \param branch_arg: current branch node
         *  \param return_leaf_arg: return pointer to leaf container
         *  \param parent_of_leaf_arg: return pointer to parent of leaf node
         *  \param branch_reset_arg: Reset pointer array of current branch
         *  \return depth mask at which leaf node was created/found
         **/
        unsigned int
        createLeafRecursive (const OctreeKey& key_arg,
                             unsigned int depth_mask_arg,
                             BranchNode* branch_arg,
                             LeafNode*& return_leaf_arg,
                             BranchNode*& parent_of_leaf_arg,
                             bool branch_reset_arg = false);


        /** \brief Recursively search for a given leaf node and return a pointer.
         *  \note  If leaf node does not exist, a 0 pointer is returned.
         *  \param key_arg: reference to an octree key
         *  \param depth_mask_arg: depth mask used for octree key analysis and for branch depth indicator
         *  \param branch_arg: current branch node
         *  \param result_arg: pointer to leaf container class
         **/
        void
        findLeafRecursive (const OctreeKey& key_arg,
                           unsigned int depth_mask_arg,
                           BranchNode* branch_arg,
                           LeafContainerT*& result_arg) const;


        /** \brief Recursively search and delete leaf node
         *  \param key_arg: reference to an octree key
         *  \param depth_mask_arg: depth mask used for octree key analysis and branch depth indicator
         *  \param branch_arg: current branch node
         *  \return "true" if branch does not contain any childs; "false" otherwise. This indicates if current branch can be deleted.
         **/
        bool
        deleteLeafRecursive (const OctreeKey& key_arg,
                             unsigned int depth_mask_arg,
                             BranchNode* branch_arg);

        /** \brief Recursively explore the octree and output binary octree description together with a vector of leaf node DataT content.
         *  \param branch_arg: current branch node
         *  \param key_arg: reference to an octree key
         *  \param binary_tree_out_arg: binary output vector
         *  \param leaf_container_vector_arg: vector to return pointers to all leaf container in the tree.
         *  \param do_XOR_encoding_arg: select if binary tree structure should be generated based on current octree (false) of based on a XOR comparison between current and previous octree
         *  \param new_leafs_filter_arg: execute callback only for leaf nodes that did not exist in preceding buffer
         **/
        void
        serializeTreeRecursive (BranchNode* branch_arg,
                                OctreeKey& key_arg,
                                std::vector<char>* binary_tree_out_arg,
                                typename std::vector<LeafContainerT*>* leaf_container_vector_arg,
                                bool new_leafs_filter_arg = false);

        /** \brief Rebuild an octree based on binary XOR octree description and DataT objects for leaf node initialization.
         *  \param branch_arg: current branch node
         *  \param depth_mask_arg: depth mask used for octree key analysis and branch depth indicator
         *  \param key_arg: reference to an octree key
         *  \param binary_tree_in_it_arg iterator of binary input data
         *  \param binary_tree_in_it_end_arg
         *  \param leaf_container_vector_it_arg: iterator pointing to leaf container pointers to be added to a leaf node
         *  \param leaf_container_vector_it_end_arg: iterator pointing to leaf container pointers pointing to last object in input container.
         *  \param branch_reset_arg: Reset pointer array of current branch
         *  \param do_XOR_decoding_arg: select if binary tree structure is based on current octree (false) of based on a XOR comparison between current and previous octree
         **/
        void
        deserializeTreeRecursive (BranchNode* branch_arg,
                                  unsigned int depth_mask_arg,
                                  OctreeKey& key_arg,
                                  typename std::vector<char>::const_iterator& binary_tree_in_it_arg,
                                  typename std::vector<char>::const_iterator& binary_tree_in_it_end_arg,
                                  typename std::vector<LeafContainerT*>::const_iterator* leaf_container_vector_it_arg,
                                  typename std::vector<LeafContainerT*>::const_iterator* leaf_container_vector_it_end_arg,
                                  bool branch_reset_arg = false);


        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Serialization callbacks
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Callback executed for every leaf node data during serialization
         **/
        virtual void serializeTreeCallback (LeafContainerT &, const OctreeKey &)
        {

        }

        /** \brief Callback executed for every leaf node data during deserialization
         **/
        virtual void deserializeTreeCallback (LeafContainerT&, const OctreeKey&)
        {

        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Helpers
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Recursively explore the octree and remove unused branch and leaf nodes
         *  \param branch_arg: current branch node
         **/
        void
        treeCleanUpRecursive (BranchNode* branch_arg,
            unsigned short buffer_selector_arg);

        /** \brief Helper function to calculate the binary logarithm
         * \param n_arg: some value
         * \return binary logarithm (log2) of argument n_arg
         */
        [[deprecated("use std::log2 instead")]]
        inline double Log2 (double n_arg)
        {
          return std::log2 (n_arg);
        }

        /** \brief Test if octree is able to dynamically change its depth. This is required for adaptive bounding box adjustment.
         *  \return "false" - not resizeable due to XOR serialization
         **/
        inline bool octreeCanResize ()
        {
          return (false);
        }

        /** \brief Prints binary representation of a byte - used for debugging
         *  \param data_arg - byte to be printed to stdout
         **/
        inline void printBinary (char data_arg)
        {
          unsigned char mask = 1;  // Bit mask

          // Extract the bits
          for (int i = 0; i < 8; i++)
          {
            // Mask each bit in the byte and print it
            std::cout << ((data_arg & (mask << i)) ? "1" : "0");
          }
          std::cout << std::endl;
        }

        void
        serializeTreeRecursive (BranchNode* branch_arg,
                                OctreeKey& key_arg,
                                unsigned short buffer_selector_arg,
                                typename std::vector<LeafContainerT*>* leaf_container_vector_arg,
                                boost::function<bool ( std::vector<bool> bit_patterns )>* function = nullptr,
                                std::vector<std::pair<unsigned char, LeafContainerT*>>* all_buffer_vector_arg = nullptr,
                                bool output_multi_buf_data = false);

        std::vector<bool> getNodeBitPattern (const BranchNode& branch_arg,
                                           unsigned char child_idx_arg);
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Globals
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief Amount of leaf nodes   **/
        std::size_t leaf_count_;
        #ifdef BUFFER_MAX
        std::size_t leaf_count_all_[BUFFER_MAX];
        #endif

        /** \brief Amount of branch nodes   **/
        std::size_t branch_count_;

        /** \brief Pointer to root branch node of octree   **/
        BranchNode* root_node_;

        /** \brief Depth mask based on octree depth   **/
        unsigned int depth_mask_;

        /** \brief key range */
        OctreeKey max_key_;

        /** \brief Currently active octree buffer  **/
        unsigned short buffer_selector_;
        #ifndef BUFFER_MAX
        unsigned short buffer_size_;
        #endif

        /** \brief Octree depth */
        unsigned int octree_depth_;

        /** \brief Enable dynamic_depth
         *  \note Note that this parameter is ignored in octree2buf! */
        bool dynamic_depth_enabled_;

    };
  }
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/octree/impl/octree2buf_base.hpp>
#endif

#ifndef PCL_OCTREE_NBUF_BASE_HPP
#define PCL_OCTREE_NBUF_BASE_HPP

namespace pcl
{
  namespace octree
  {
    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT>
    OctreeNBufBase<LeafContainerT, BranchContainerT>::OctreeNBufBase () :
      root_node_ (new BranchNode ()),
      leaf_count_(0),
      branch_count_(1),
      depth_mask_ (0), 
      max_key_ (),
      buffer_selector_ (0),
      octree_depth_ (0),
      dynamic_depth_enabled_(false)
    {
      #ifdef BUFFER_MAX
      memset(leaf_count_all_, 0, sizeof(leaf_count_all_));
      #endif
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT>
    OctreeNBufBase<LeafContainerT, BranchContainerT>::~OctreeNBufBase ()
    {
      // deallocate tree structure
      deleteTree ();
      if(root_node_)
      {
        delete (root_node_);
        root_node_ = nullptr;
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT> void
    OctreeNBufBase<LeafContainerT, BranchContainerT>::setMaxVoxelIndex (unsigned int max_voxel_index_arg)
    {
      unsigned int treeDepth;

      assert (max_voxel_index_arg > 0);

      // tree depth == amount of bits of maxVoxels
      treeDepth = std::max ((std::min (static_cast<unsigned int> (OctreeKey::maxDepth),
                                       static_cast<unsigned int> (std::ceil (std::log2 (max_voxel_index_arg))))),
                                       static_cast<unsigned int> (0));

      // define depthMask_ by setting a single bit to 1 at bit position == tree depth
      depth_mask_ = (1 << (treeDepth - 1));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT> void
    OctreeNBufBase<LeafContainerT, BranchContainerT>::setTreeDepth (unsigned int depth_arg)
    {
      assert (depth_arg > 0);

      // set octree depth
      octree_depth_ = depth_arg;

      // define depthMask_ by setting a single bit to 1 at bit position == tree depth
      depth_mask_ = (1 << (depth_arg - 1));

      // define max. keys
      max_key_.x = max_key_.y = max_key_.z = (1 << depth_arg) - 1;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
     template<typename LeafContainerT, typename BranchContainerT>  LeafContainerT*
     OctreeNBufBase<LeafContainerT, BranchContainerT>::findLeaf (unsigned int idx_x_arg, unsigned int idx_y_arg, unsigned int idx_z_arg)
     {
       // generate key
       OctreeKey key (idx_x_arg, idx_y_arg, idx_z_arg);

       // check if key exist in octree
       return ( findLeaf (key));
     }

    //////////////////////////////////////////////////////////////////////////////////////////////
     template<typename LeafContainerT, typename BranchContainerT>  LeafContainerT*
     OctreeNBufBase<LeafContainerT, BranchContainerT>::createLeaf (unsigned int idx_x_arg, unsigned int idx_y_arg, unsigned int idx_z_arg)
     {
       // generate key
       OctreeKey key (idx_x_arg, idx_y_arg, idx_z_arg);

       // check if key exist in octree
       return ( createLeaf (key));
     }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT> bool
    OctreeNBufBase<LeafContainerT, BranchContainerT>::existLeaf (unsigned int idx_x_arg, unsigned int idx_y_arg, unsigned int idx_z_arg) const
    {
      // generate key
      OctreeKey key (idx_x_arg, idx_y_arg, idx_z_arg);

      // check if key exist in octree
      return existLeaf (key);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT> void
    OctreeNBufBase<LeafContainerT, BranchContainerT>::removeLeaf (unsigned int idx_x_arg, unsigned int idx_y_arg, unsigned int idx_z_arg)
    {
      // generate key
      OctreeKey key (idx_x_arg, idx_y_arg, idx_z_arg);

      // free voxel at key
      return (this->removeLeaf (key));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT> void
    OctreeNBufBase<LeafContainerT, BranchContainerT>::deleteTree ()
    {
      if (root_node_)
      {
        // reset octree
        for(unsigned short selector = 0; selector < BUFFER_MAX; selector++)
        {
          switchBuffers(selector);
          leaf_count_all_[selector] = 0;
        }
        branch_count_ = 1;

        depth_mask_ = 0;
        octree_depth_ = 0;
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT> void
    OctreeNBufBase<LeafContainerT, BranchContainerT>::switchBuffers (unsigned short buffer_selector_arg)
    {
      // make sure that all unused branch nodes from previous buffer are deleted
      treeCleanUpRecursive (root_node_, buffer_selector_arg);

      // switch butter selector
      leaf_count_all_[buffer_selector_arg] = leaf_count_;
      buffer_selector_ = buffer_selector_arg;

      // reset flags
      leaf_count_ = 0;
      leaf_count_all_[buffer_selector_] = leaf_count_;
      branch_count_ = 1;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT> void
    OctreeNBufBase<LeafContainerT, BranchContainerT>::serializeTree (std::vector<char>& binary_tree_out_arg)
    {
      OctreeKey new_key;
      
      // clear binary vector
      binary_tree_out_arg.clear ();
      binary_tree_out_arg.reserve (branch_count_);

      serializeTreeRecursive (root_node_, new_key, &binary_tree_out_arg, nullptr, false);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT> void
    OctreeNBufBase<LeafContainerT, BranchContainerT>::serializeTree (std::vector<char>& binary_tree_out_arg,
                                                                     std::vector<LeafContainerT*>& leaf_container_vector_arg)
    {
      OctreeKey new_key;

      // clear output vectors
      binary_tree_out_arg.clear ();
      leaf_container_vector_arg.clear ();

      leaf_container_vector_arg.reserve (leaf_count_all_);
      binary_tree_out_arg.reserve (branch_count_);

      serializeTreeRecursive (root_node_, new_key, &binary_tree_out_arg, &leaf_container_vector_arg, false);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT> void
    OctreeNBufBase<LeafContainerT, BranchContainerT>::serializeLeafs (std::vector<LeafContainerT*>& leaf_container_vector_arg)
    {
      OctreeKey new_key;

      // clear output vector
      leaf_container_vector_arg.clear ();

      leaf_container_vector_arg.reserve (leaf_count_all_[buffer_selector_]);

      serializeTreeRecursive (root_node_, new_key, nullptr, &leaf_container_vector_arg, false);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT> void
    OctreeNBufBase<LeafContainerT, BranchContainerT>::deserializeTree (std::vector<char>& binary_tree_in_arg)
    {
      OctreeKey new_key;

      // we will rebuild an octree -> reset leafCount
      leaf_count_all_[buffer_selector_] = 0;

      // iterator for binary tree structure vector
      std::vector<char>::const_iterator binary_tree_in_it = binary_tree_in_arg.begin ();
      std::vector<char>::const_iterator binary_tree_in_it_end = binary_tree_in_arg.end ();

      deserializeTreeRecursive (root_node_, depth_mask_, new_key,
          binary_tree_in_it, binary_tree_in_it_end, nullptr, nullptr, false);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT> void
    OctreeNBufBase<LeafContainerT, BranchContainerT>::deserializeTree (std::vector<char>& binary_tree_in_arg,
                                                                       std::vector<LeafContainerT*>& leaf_container_vector_arg)
    {
      OctreeKey new_key;

      // set data iterator to first element
      typename std::vector<LeafContainerT*>::const_iterator leaf_container_vector_it = leaf_container_vector_arg.begin ();

      // set data iterator to last element
      typename std::vector<LeafContainerT*>::const_iterator leaf_container_vector_it_end = leaf_container_vector_arg.end ();

      // we will rebuild an octree -> reset leafCount
      leaf_count_all_[buffer_selector_] = 0;

      // iterator for binary tree structure vector
      std::vector<char>::const_iterator binary_tree_in_it = binary_tree_in_arg.begin ();
      std::vector<char>::const_iterator binary_tree_in_it_end = binary_tree_in_arg.end ();

      deserializeTreeRecursive (root_node_,
                                depth_mask_,
                                new_key,
                                binary_tree_in_it,
                                binary_tree_in_it_end,
                                &leaf_container_vector_it,
                                &leaf_container_vector_it_end,
                                false);
    }


    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT> void
    OctreeNBufBase<LeafContainerT, BranchContainerT>::serializeNewLeafs (std::vector<LeafContainerT*>& leaf_container_vector_arg)
    {
      OctreeKey new_key;

      // clear output vector
      leaf_container_vector_arg.clear ();
      leaf_container_vector_arg.reserve (leaf_count_all_[buffer_selector_]);

      serializeTreeRecursive (root_node_, new_key, nullptr, &leaf_container_vector_arg, true);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT>
      unsigned int
      OctreeNBufBase<LeafContainerT, BranchContainerT>::createLeafRecursive (const OctreeKey& key_arg,
                                                                             unsigned int depth_mask_arg,
                                                                             BranchNode* branch_arg,
                                                                             LeafNode*& return_leaf_arg,
                                                                             BranchNode*& parent_of_leaf_arg,
                                                                             bool branch_reset_arg)
      {
      // find branch child from key
      unsigned char child_idx = key_arg.getChildIdxWithDepthMask (depth_mask_arg);

      if (depth_mask_arg > 1)
      {
        // we have not reached maximum tree depth
        BranchNode* child_branch = nullptr;
        
        // if required branch does not exist
        if (!branch_arg->hasChild(buffer_selector_, child_idx))
        {
          bool created = false;
          
          #ifdef BUFFER_MAX
          for(unsigned short selector = 0; selector < BUFFER_MAX; selector++)
          #else
          for(unsigned short selector = 0; selector < buffer_size_; selector++)
          #endif
          {
            // check if we find a branch node reference in previous buffer
            if (branch_arg->hasChild(selector, child_idx))
            {
              OctreeNode* child_node = branch_arg->getChildPtr(selector,child_idx);

              if (child_node->getNodeType()==BRANCH_NODE) {
                child_branch = static_cast<BranchNode*> (child_node);
                branch_arg->setChildPtr(buffer_selector_, child_idx, child_branch);
                break;
              }
            }
          }
          if(!child_branch)
          {
            child_branch = createBranchChild (*branch_arg, child_idx);
            branch_count_++;
          }
        }
        // required branch node already exists - use it
        else
          child_branch = static_cast<BranchNode*> (branch_arg->getChildPtr(buffer_selector_,child_idx));
        
        // recursively proceed with indexed child branch
        return createLeafRecursive (key_arg, depth_mask_arg / 2, child_branch, return_leaf_arg, parent_of_leaf_arg);
      }
      else
      {
        // branch childs are leaf nodes
        LeafNode* child_leaf;
        if (!branch_arg->hasChild(buffer_selector_, child_idx))
        {
          child_leaf = createLeafChild (*branch_arg, child_idx);
          leaf_count_all_[buffer_selector_]++;
          
          // return leaf node
          return_leaf_arg = child_leaf;
          parent_of_leaf_arg = branch_arg;
        }
        else
        {
          // leaf node already exist
          return_leaf_arg = static_cast<LeafNode*> (branch_arg->getChildPtr(buffer_selector_,child_idx));;
          parent_of_leaf_arg = branch_arg;
        }
      }

      return depth_mask_arg;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT> void
    OctreeNBufBase<LeafContainerT, BranchContainerT>::findLeafRecursive (const OctreeKey& key_arg,
                                                                         unsigned int depth_mask_arg,
                                                                         BranchNode* branch_arg,
                                                                         LeafContainerT*& result_arg) const
    {
      // return leaf node
      unsigned char child_idx;

      // find branch child from key
      child_idx = key_arg.getChildIdxWithDepthMask (depth_mask_arg);

      if (depth_mask_arg > 1)
      {
        // we have not reached maximum tree depth
        BranchNode* child_branch;
        child_branch = static_cast<BranchNode*> (branch_arg->getChildPtr(buffer_selector_,child_idx));
        
        if (child_branch)
          // recursively proceed with indexed child branch
          findLeafRecursive (key_arg, depth_mask_arg / 2, child_branch, result_arg);
      }
      else
      {
        // we reached leaf node level
        if (branch_arg->hasChild(buffer_selector_, child_idx))
        {
          // return existing leaf node
          LeafNode* leaf_node = static_cast<LeafNode*> (branch_arg->getChildPtr(buffer_selector_,child_idx));
          result_arg = leaf_node->getContainerPtr();;
        }
      }    
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT> bool
    OctreeNBufBase<LeafContainerT, BranchContainerT>::deleteLeafRecursive (const OctreeKey& key_arg,
                                                                           unsigned int depth_mask_arg,
                                                                           BranchNode* branch_arg)
    {
      // index to branch child
      unsigned char child_idx;
      // indicates if branch is empty and can be safely removed
      bool bNoChilds;

      // find branch child from key
      child_idx = key_arg.getChildIdxWithDepthMask (depth_mask_arg);

      if (depth_mask_arg > 1)
      {
        // we have not reached maximum tree depth
        BranchNode* child_branch;
        bool bBranchOccupied;
        
        // next branch child on our path through the tree
        child_branch = static_cast<BranchNode*> (branch_arg->getChildPtr(buffer_selector_,child_idx));
        
        if (child_branch)
        {
          // recursively explore the indexed child branch
          bBranchOccupied = deleteLeafRecursive (key_arg, depth_mask_arg / 2, child_branch);
          
          if (!bBranchOccupied)
          {
            // child branch does not own any sub-child nodes anymore -> delete child branch
            deleteBranchChild (*branch_arg, buffer_selector_, child_idx);
            branch_count_--;
          }
        }
      }
      else
      {
        // our child is a leaf node -> delete it
        deleteBranchChild (*branch_arg, buffer_selector_, child_idx);
        leaf_count_all_[buffer_selector_]--;
      }

      // check if current branch still owns childs
      bNoChilds = false;
      for (child_idx = 0; child_idx < 8; child_idx++)
      {
        bNoChilds = branch_arg->hasChild(buffer_selector_, child_idx);
        if (bNoChilds)
          break;
      }

      // return true if current branch can be deleted
      return (bNoChilds);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT> void OctreeNBufBase<
        LeafContainerT, BranchContainerT>::serializeTreeRecursive (BranchNode* branch_arg,
                                                                   OctreeKey& key_arg,
                                                                   std::vector<char>* binary_tree_out_arg,
                                                                   typename std::vector<LeafContainerT*>* leaf_container_vector_arg,
                                                                   bool new_leafs_filter_arg)
    {
      // bit pattern
      char branch_bit_pattern_curr_buffer;

      // occupancy bit patterns of branch node  (current and previous octree buffer)
      branch_bit_pattern_curr_buffer = getBranchBitPattern (*branch_arg, buffer_selector_);
      if (binary_tree_out_arg)
      {
        // write bit pattern of current buffer to output vector
        binary_tree_out_arg->push_back (branch_bit_pattern_curr_buffer);
      }

      // iterate over all children
      for (unsigned char child_idx = 0; child_idx < 8; child_idx++)
      {
        if (branch_arg->hasChild(buffer_selector_, child_idx))
        {
          // add current branch voxel to key
          key_arg.pushBranch(child_idx);
          
          OctreeNode *child_node = branch_arg->getChildPtr(buffer_selector_,child_idx);
          
          switch (child_node->getNodeType ())
          {
            case BRANCH_NODE:
            {
                // recursively proceed with indexed child branch
                serializeTreeRecursive (static_cast<BranchNode*> (child_node), key_arg, binary_tree_out_arg,
                                        leaf_container_vector_arg, new_leafs_filter_arg);
                break;
            }
            case LEAF_NODE:
            {
              LeafNode* child_leaf = static_cast<LeafNode*> (child_node);

              if (new_leafs_filter_arg)
                {
                  bool is_new = true;
                  #ifdef BUFFER_MAX
                  for(unsigned short selector = 0; selector < BUFFER_MAX; selector++)
                  #else
                  for(unsigned short selector = 0; selector < buffer_size_; selector++)
                  #endif
                  {
                    if(selector == buffer_selector_) continue;
                    if (branch_arg->hasChild (selector, child_idx)) {
                      is_new = false;
                    }
                  }
                  if (is_new)
                  {
                    if (leaf_container_vector_arg)
                      leaf_container_vector_arg->push_back(child_leaf->getContainerPtr());

                    serializeTreeCallback (**child_leaf, key_arg);
                  }
              } else
              {

                if (leaf_container_vector_arg)
                  leaf_container_vector_arg->push_back(child_leaf->getContainerPtr());

                serializeTreeCallback (**child_leaf, key_arg);
              }

              break;
            }
            default:
              break;
          }

          // pop current branch voxel from key
          key_arg.popBranch();
        }
        else 
        {
          #ifdef BUFFER_MAX
          for(unsigned short selector = 0; selector < BUFFER_MAX; selector++)
          #else
          for(unsigned short selector = 0; selector < buffer_size_; selector++)
          #endif
          {
            if(selector == buffer_selector_) continue;
            if (branch_arg->hasChild (selector, child_idx))
            {
              // delete branch, free memory
              deleteBranchChild (*branch_arg, selector, child_idx);
            }
          }
        }
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT> void OctreeNBufBase<
        LeafContainerT, BranchContainerT>::expendBufferRecursive (BranchNode* branch_arg, unsigned short buffer_selector_arg)
    {
      // iterate over all children
      for (unsigned char child_idx = 0; child_idx < 8; child_idx++)
      {
        if (branch_arg->hasChild(buffer_selector_arg, child_idx))
        {
          OctreeNode *child_node = branch_arg->getChildPtr(buffer_selector_arg,child_idx);
          
          switch (child_node->getNodeType ())
          {
            case BRANCH_NODE:
            {
                // recursively proceed with indexed child branch
                expendBufferRecursive (static_cast<BranchNode*> (child_node), buffer_selector_arg);
                break;
            }
            case LEAF_NODE:
            {
              
              break;
            }
            default:
              break;
          }
        }
      }
    }


    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT> void
    OctreeNBufBase<LeafContainerT, BranchContainerT>::deserializeTreeRecursive (BranchNode* branch_arg,
        unsigned int depth_mask_arg, OctreeKey& key_arg,
        typename std::vector<char>::const_iterator& binaryTreeIT_arg,
        typename std::vector<char>::const_iterator& binaryTreeIT_End_arg,
        typename std::vector<LeafContainerT*>::const_iterator* dataVectorIterator_arg,
        typename std::vector<LeafContainerT*>::const_iterator* dataVectorEndIterator_arg,
        bool branch_reset_arg)
    {
      // node bits
      char nodeBits;
      char recoveredNodeBits;

      // branch reset -> this branch has been taken from previous buffer
      if (branch_reset_arg)
      {
        // we can safely remove children references
        for (unsigned char child_idx = 0; child_idx < 8; child_idx++)
        {
          branch_arg->setChildPtr(buffer_selector_, child_idx, nullptr);
        }  
      }

      if (binaryTreeIT_arg!=binaryTreeIT_End_arg) {
        // read branch occupancy bit pattern from vector
        nodeBits = *binaryTreeIT_arg++;

        recoveredNodeBits = nodeBits;

        // iterate over all children
        for (unsigned char child_idx = 0; child_idx < 8; child_idx++)
        {
          // if occupancy bit for child_idx is set..
          if (recoveredNodeBits & (1 << child_idx))
          {
            // add current branch voxel to key
            key_arg.pushBranch(child_idx);

            bool doNodeReset;
            
            doNodeReset = false;
            
            if (depth_mask_arg > 1)
            {
              // we have not reached maximum tree depth

              BranchNode* child_branch;

              // check if we find a branch node reference in previous buffer
              if (!branch_arg->hasChild(buffer_selector_, child_idx))
              {
                #ifdef BUFFER_MAX
                for(unsigned short selector = 0; selector < BUFFER_MAX; selector++)
                #else
                for(unsigned short selector = 0; selector < buffer_size_; selector++)
                #endif
                {
                  if(selector == buffer_selector_) continue;
                  if (branch_arg->hasChild(selector, child_idx))
                  {
                    OctreeNode* child_node = branch_arg->getChildPtr(selector,child_idx);

                    if (child_node->getNodeType()==BRANCH_NODE) {
                      child_branch = static_cast<BranchNode*> (child_node);
                      branch_arg->setChildPtr(buffer_selector_, child_idx, child_node);
                    } else {
                      // depth has changed.. child in preceding buffer is a leaf node.
                      deleteBranchChild (*branch_arg, selector, child_idx);
                      child_branch = createBranchChild (*branch_arg, child_idx);
                    }

                    // take child branch from previous buffer
                    doNodeReset = true; // reset the branch pointer array of stolen child node
                  }
                  else
                  {
                    // if required branch does not exist -> create it
                    child_branch = createBranchChild (*branch_arg, child_idx);
                  }
                }

                branch_count_++;

              }
              else
              {
                // required branch node already exists - use it
                child_branch = static_cast<BranchNode*> (branch_arg->getChildPtr(buffer_selector_,child_idx));
              }

              // recursively proceed with indexed child branch
              deserializeTreeRecursive (child_branch, depth_mask_arg / 2, key_arg,
                  binaryTreeIT_arg, binaryTreeIT_End_arg,
                  dataVectorIterator_arg, dataVectorEndIterator_arg,
                  doNodeReset);

            }
            else
            {
              // branch childs are leaf nodes
              LeafNode* child_leaf;
              
              #ifdef BUFFER_MAX
              for(unsigned short selector = 0; selector < BUFFER_MAX; selector++)
              #else
              for(unsigned short selector = 0; selector < buffer_size_; selector++)
              #endif
              {
                if(selector == buffer_selector_) continue;
                // check if we can take copy a reference pointer from previous buffer
                if (branch_arg->hasChild(selector, child_idx))
                {
                  // take child leaf node from previous buffer
                  OctreeNode* child_node = branch_arg->getChildPtr(selector,child_idx);
                  if (child_node->getNodeType()==LEAF_NODE) {
                    child_leaf = static_cast<LeafNode*> (child_node);
                    branch_arg->setChildPtr(buffer_selector_, child_idx, child_node);
                  } else {
                    // depth has changed.. child in preceding buffer is a leaf node.
                    deleteBranchChild (*branch_arg, selector, child_idx);
                    child_leaf = createLeafChild (*branch_arg, child_idx);
                  }
                }
                else
                {
                  // if required leaf does not exist -> create it
                  child_leaf = createLeafChild (*branch_arg, child_idx);
                }
              }

              // we reached leaf node level

              if (dataVectorIterator_arg
                  && (*dataVectorIterator_arg != *dataVectorEndIterator_arg))
              {
                LeafContainerT& container = **child_leaf;
                container =  ***dataVectorIterator_arg;
                ++*dataVectorIterator_arg;
              }

              leaf_count_all_[buffer_selector_]++;

              // execute deserialization callback
              deserializeTreeCallback (**child_leaf, key_arg);

            }

            // pop current branch voxel from key
            key_arg.popBranch();
          }
          else 
          {
            #ifdef BUFFER_MAX
            for(unsigned short selector = 0; selector < BUFFER_MAX; selector++)
            #else
            for(unsigned short selector = 0; selector < buffer_size_; selector++)
            #endif
            {
              if(selector == buffer_selector_) continue;
              if (branch_arg->hasChild (selector, child_idx))
              {
                // remove old branch pointer information in current branch
                branch_arg->setChildPtr(buffer_selector_, child_idx, nullptr);
                
                // remove unused branches in previous buffer
                deleteBranchChild (*branch_arg, selector, child_idx);
              }
            }
          }
        }
      }

    }
    
    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT> void
    OctreeNBufBase<LeafContainerT, BranchContainerT>::treeCleanUpRecursive (BranchNode* branch_arg,
            unsigned short buffer_selector_arg)
    {
      // iterate over all children
      for (unsigned char child_idx = 0; child_idx < 8; child_idx++)
      {
        if (branch_arg->hasChild(buffer_selector_arg, child_idx))
        {
          OctreeNode *child_node = branch_arg->getChildPtr(buffer_selector_arg,child_idx);
          switch (child_node->getNodeType ())
          {
            case BRANCH_NODE:
            {
              bool do_delete = true;
              // recursively proceed with indexed child branch
              treeCleanUpRecursive (static_cast<BranchNode*> (child_node), buffer_selector_arg);
              for(unsigned short selector = 0; selector < BUFFER_MAX; selector++)
              {
                for(unsigned char child_idx2 = 0; child_idx2 < 8; child_idx2++) 
                {
                  if(static_cast<BranchNode*> (child_node)->hasChild(selector, child_idx2))
                  {
                    do_delete = false;
                    break;
                  }
                }
              }
              if(do_delete) {
                delete static_cast<BranchNode*>(child_node);
              }
              branch_arg->setChildPtr(buffer_selector_arg, child_idx, nullptr);
              break;
            }
            case LEAF_NODE:
              delete static_cast<LeafNode*>(child_node);
              branch_arg->setChildPtr(buffer_selector_arg, child_idx, nullptr);
              // leaf level - nothing to do..
              break;
            default:
              break;
          }
        }
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT> void
    OctreeNBufBase<LeafContainerT, BranchContainerT>::serializeLeafs (std::vector<LeafContainerT*>& leaf_container_vector_arg,
                                                                      unsigned short buffer_selector_arg,
                                                                      boost::function<bool ( std::vector<bool> bit_patterns )>* function,
                                                                      std::vector<std::pair<unsigned char, LeafContainerT*>>* all_buffer_vector_arg,
                                                                      bool output_multi_buf_data)
    {
      OctreeKey new_key;
      // clear output vector
      leaf_container_vector_arg.clear ();
      leaf_container_vector_arg.reserve (leaf_count_all_[buffer_selector_]);

      serializeTreeRecursive (root_node_, new_key, buffer_selector_arg, &leaf_container_vector_arg, function, all_buffer_vector_arg, output_multi_buf_data);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename LeafContainerT, typename BranchContainerT> void OctreeNBufBase<
        LeafContainerT, BranchContainerT>::serializeTreeRecursive (BranchNode* branch_arg,
                                                                   OctreeKey& key_arg,
                                                                   unsigned short buffer_selector_arg,
                                                                   typename std::vector<LeafContainerT*>* leaf_container_vector_arg,
                                                                   boost::function<bool ( std::vector<bool> bit_patterns )>* function,
                                                                   std::vector<std::pair<unsigned char, LeafContainerT*>>* all_buffer_vector_arg,
                                                                   bool output_multi_buf_data)
    {
      // iterate over all children
      for (unsigned char child_idx = 0; child_idx < 8; child_idx++)
      {
        key_arg.pushBranch(child_idx);
        if(all_buffer_vector_arg)
        {
          for(unsigned short selector = 0; selector < BUFFER_MAX; selector++)
          {
            if (branch_arg->hasChild(selector, child_idx))
            {
              bool do_break = !output_multi_buf_data;
              OctreeNode *child_node = branch_arg->getChildPtr(selector,child_idx);
              
              switch (child_node->getNodeType ())
              {
                case BRANCH_NODE:
                {
                    // recursively proceed with indexed child branch
                    serializeTreeRecursive (static_cast<BranchNode*> (child_node),
                                            key_arg,
                                            buffer_selector_arg,
                                            leaf_container_vector_arg,
                                            function,
                                            all_buffer_vector_arg,
                                            output_multi_buf_data);
                    do_break = true;
                    break;
                }
                case LEAF_NODE:
                {
                  LeafNode* child_leaf = static_cast<LeafNode*> (child_node);

                  if (leaf_container_vector_arg) 
                  {
                    bool keep = true;
                    if(function)
                    {
                      std::vector<bool> nodes_bit = getNodeBitPattern(*branch_arg, child_idx);
                      keep = (*function)(nodes_bit);
                    }
                    if(keep)
                    {
                      bool exists = false;
                      for(auto &other_buffer_arg : *all_buffer_vector_arg)
                      {
                        if(other_buffer_arg.first == selector)
                        {
                          std::vector<int> indicesVector;
                          child_leaf->getContainerPtr()->getPointIndices(indicesVector);
                          for(auto &idx : indicesVector)
                          other_buffer_arg.second->addPointIndex(idx);
                          exists = true;
                        }
                      }
                      if(!exists)
                        all_buffer_vector_arg->push_back({selector, child_leaf->getContainerPtr()->deepCopy()});
                    }
                  }
                  break;
                }
                default:
                  break;
              }
              if(do_break) break;
            }
          }
        }
        else
        {
          if (branch_arg->hasChild(buffer_selector_arg, child_idx))
          {
            OctreeNode *child_node = branch_arg->getChildPtr(buffer_selector_arg,child_idx);
            
            switch (child_node->getNodeType ())
            {
              case BRANCH_NODE:
              {
                  // recursively proceed with indexed child branch
                  serializeTreeRecursive (static_cast<BranchNode*> (child_node),
                                          key_arg,
                                          buffer_selector_arg,
                                          leaf_container_vector_arg,
                                          function,
                                          all_buffer_vector_arg,
                                          output_multi_buf_data);
                  break;
              }
              case LEAF_NODE:
              {
                LeafNode* child_leaf = static_cast<LeafNode*> (child_node);

                if (leaf_container_vector_arg) 
                {
                  bool keep = true;
                  if(function)
                  {
                    std::vector<bool> nodes_bit = getNodeBitPattern(*branch_arg, child_idx);
                    keep = (*function)(nodes_bit);
                  }
                  if(keep)
                  {
                    leaf_container_vector_arg->push_back(child_leaf->getContainerPtr());
                  }
                }
                break;
              }
              default:
                break;
            }
          }
        }
        
        key_arg.popBranch();
      }
    }
    template<typename LeafContainerT, typename BranchContainerT> std::vector<bool> OctreeNBufBase<
        LeafContainerT, BranchContainerT>::getNodeBitPattern (const BranchNode& branch_arg,
                                           unsigned char child_idx_arg)
    {
      std::vector<bool> nodes_bit;

      // create bit pattern
      for (unsigned short selector = 0; selector < BUFFER_MAX; selector++)
      {
        const OctreeNode* child = branch_arg.getChildPtr(selector, child_idx_arg);
        nodes_bit.push_back(!!child);
      }

      return nodes_bit;
    }

  }
}

#define PCL_INSTANTIATE_OctreeNBufBase(T) template class PCL_EXPORTS pcl::octree::OctreeNBufBase<T>;

#endif

