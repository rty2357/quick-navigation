/*
 * gnd-coord-tree.hpp
 *
 *  Created on: 2011/06/16
 *      Author: tyamada
 */

#ifndef GND_COORD_TREE_HPP_
#define GND_COORD_TREE_HPP_

#include "gnd-matrix-base.hpp"
#include "gnd-queue.hpp"


// ---> type declaration
namespace gnd {
	namespace matrix {
		class coord_tree;
		struct coord_node;
	}
}
// <--- type declaration

// ---> coordinate tree node structure definition
namespace gnd { // ---> namespace gnd
	namespace matrix { // ---> namespace matrix

		/**
		 * @brief node of coordinate tree
		 */
		struct coord_node : public coord_matrix {
			static const size_t NameLength = 32;	///< name buffer size

			int id;									///< cooridnate id
			int depth;								///< tree depth
			int parent;								///< parent node index
			char name[NameLength];					///< name of coordinate

			// ---> constructor
			coord_node();
			coord_node( const flex* obj );
			coord_node( const coord_matrix* obj );
			coord_node( const coord_node* obj );
			// <--- constructor
		};
	}
}
// <--- coordinate tree node structure definition


// ---> coordinate tree node structure definition
namespace gnd {
	namespace matrix {

		/**
		 * @brief coordinate tree
		 */
		class coord_tree {
			// ---> constructor, destructor
		public:
			coord_tree();
			~coord_tree();

			// ---> initial
		public:
			void initial();

			// ---> constant definition
		public:
			static const int InvalidID = -1;
			static const int InvalidDepth = -1;
			static const int InvalidParent = -1;

			// ---> typedef
		public:
			typedef coord_node					node_t;		///< coorinate tree node type
			typedef queue<node_t>				tree_t;		///< coorinate tree storage

			// ---> node list
		private:
			tree_t _tree;						///< coorinate tree
			int _id;							///< issued id No

			// ---> create coordinate
		private:
			int create_root(node_t *rt);
		public:
			static const int RootID = 0;		///< coorinate tree root id
			static const int RootDepth = 0;		///< coorinate tree root depth
			// ---> setter
		public:
			template< typename MTRX >
			int add( const char* n, const char* p, const MTRX *c );
			template< typename MTRX >
			int add_node( const char* n, const int p, const MTRX *c );

			// ---> set_mapping
		public:
			// setter
			template< typename MTRX >
			int set_coordinate( const int key, const MTRX *o );
			template< typename MTRX >
			int set_coordinate( const char* key, const MTRX *o );

			// ---> find node
		public:
			int find(const char* n);
			int find(const int id);

			// ---> get
		public:
			template< typename T1, typename T2, typename MTRX >
			int get_convert_matrix( const T1& f, const T2& t, MTRX *m );

			int get_node_id(const char* n);
		};

	}
}



namespace gnd {
	namespace matrix {
		/**
		 * @brief constructor
		 */
		inline
		coord_node::coord_node()
		: id(coord_tree::InvalidID), depth(coord_tree::InvalidDepth), parent(coord_tree::InvalidParent)
		{
			::memset(name, 0, sizeof(name));
		}

		/**
		 * @brief constructor
		 * @param coordinate matrix
		 */
		inline
		coord_node::coord_node( const flex *c)
		: id(coord_tree::InvalidID), depth(coord_tree::InvalidDepth), parent(coord_tree::InvalidParent)
		{
			::memset(name, 0, sizeof(name));
			matrix::copy(this, c);
		}

		/*
		 * @brief copy constructor
		 */
		inline coord_node::coord_node( const coord_matrix* c )
		: id(coord_tree::InvalidID), depth(coord_tree::InvalidDepth), parent(coord_tree::InvalidParent)
		{
			::memset(name, 0, sizeof(name));
			matrix::copy(this, c);
		}
	} // <--- namespace matrix
} // <--- namespace gnd



namespace gnd {
	namespace matrix {
		// ---> constructor, destructor
		/**
		 * @brief constructor
		 */
		inline
		coord_tree::coord_tree()
		{
			initial();
		}

		/**
		 * @brief destructor
		 */
		inline
		coord_tree::~coord_tree()
		{

		}


		/**
		 * @brief initialize
		 */
		inline
		void coord_tree::initial()
		{
			node_t root;

			{ // ---> issue id to root node
				_id = RootID;
				root.id = _id;
			} // <--- issue id to root node

			// set name
			::strcpy(root.name, "root");
			// set root no parent node index
			root.parent = InvalidParent;
			// set root depth
			root.depth = RootDepth;
			// set root coordinate matrix(unit matirx)
			set_unit(&root);
			// put in cooridnate tree
			_tree.push_back( &root );
		}

		// ------------------------------------------------------------> YP_COORDINATE_MANAGER_CLASS : node list
		/**
		 * @brief add coordinate node
		 * @param [in] n : added coordinate name
		 * @param [in] p : parent node name
		 * @param [in] o : added cooridnate matrix
		 * @return    <0 : fail to node addition
		 *           >=0 : added node id
		 */
		template< typename MTRX >
		inline
		int coord_tree::add( const char* n, const char* p, const MTRX *o )
		{
			node_t node;

			{ // ---> set parent
				int pi;	// parent index
				if( (pi = find(p)) < 0){
					return -1;
				}
				node.parent = pi;
				node.depth = _tree[pi].depth + 1;
			} // <--- set parent

			{ // ---> set coordinate convert mantrix
				matrix::copy(&node, o);
			} // <--- set coordinate convert mantrix

			{ // ---> set identification
				::strcpy(node.name, n);
				node.id = ++_id;
			} // <--- set identification

			{ // ---> set in tree
				_tree.push_back(&node);
			} // <--- set in tree

			return _id;
		}



		/**
		 * @brief add coordinate node
		 * @param [in] n : added coordinate name
		 * @param [in] p : parent node id
		 * @param [in] o : added cooridnate matrix
		 * @return    <0 : fail to node addition
		 *           >=0 : added node id
		 */
		template< typename MTRX >
		inline
		int coord_tree::add_node( const char* n, const int p, const MTRX *o )
		{
			node_t node;

			{ // ---> set parent
				int pi;	// parent index
				if( (pi = find(p)) < 0){
					return -1;
				}
				node.parent = pi;
				node.depth = _tree[pi].depth + 1;
			} // <--- set parent

			{ // ---> set coordinate convert mantrix
				matrix::copy(&node, o);
			} // <--- set coordinate convert mantrix

			{ // ---> set identification
				::strcpy(node.name, n);
				node.id = ++_id;
			} // <--- set identification

			{ // ---> set in tree
				_tree.push_back(&node);
			} // <--- set in tree

			return _id;
		}




		// ------------------------------------------------------------> set_mapping
		/**
		 * @brief set coordinate
		 * @param [in] p : node id
		 * @param [in] o : cooridnate matrix
		 * @return    <0 : fail to find node
		 */
		template< typename MTRX >
		inline
		int coord_tree::set_coordinate( const int id, const MTRX *o )
		{
			int pi;
			if( !(pi = find(id)) ){
				return -1;
			}
			// set coordinate matrix
			matrix::copy(&_tree[pi], o);

			return 0;
		}

		/**
		 * @brief set coordinate
		 * @param [in] n : node name
		 * @param [in] o : cooridnate matrix
		 * @return    <0 : fail to find node
		 */
		template< typename MTRX >
		inline
		int coord_tree::set_coordinate( const char *n, const MTRX *o )
		{
			int pi;
			if( !(pi = find(n)) ){
				return -1;
			}
			// set coordinate matrix
			matrix::copy(&_tree[pi], o);

			return 0;
		}


		// ------------------------------------------------------------> YP_COORDINATE_MANAGER_CLASS : find
		/**
		 * @brief find coordinate node
		 * @param [in] n : node name
		 * @return   >=0 : node index
		 * @return    <0 : fail to find node
		 */
		inline
		int coord_tree::find(const char *n)
		{
			uint64_t i;
			for( i = 0; i < _tree.size(); i++){
				if( _tree[i].id != InvalidID && ::strcmp(_tree[i].name, n) == 0 )	return i;
			}
			return -1;
		}

		/**
		 * @brief find coordinate node
		 * @param [in] id : node id
		 * @return    >=0 : node index
		 * @return     <0 : fail to find node
		 */
		inline
		int coord_tree::find(const int id)
		{
			uint64_t i;
			for( i = 0; i < _tree.size(); i++){
				if( _tree[i].id == id )	return i;
			}
			return -1;
		}



		/**
		 * @brief get cooridnate convert matrix
		 * @param  [in] f : from (name or id)
		 * @param  [in] t : to   (name or id)
		 * @param [out] m : coordinate convert matrix from f to t
		 * @return    < 0 : fail to find node(f or t)
		 */
		template< typename T1, typename T2, typename MTRX >
		inline
		int coord_tree::get_convert_matrix( const T1& f, const T2& t, MTRX *m )
		{
			matrix::fixed< 4, 4 > fconv;
			matrix::fixed< 4, 4 > tconv;
			matrix::fixed< 4, 4 > ws;		// work space
			int fi, ti;						// traversal node index (starting form "f" and "t")

			// find from-cooridnate
			fi = find( f );
			if( fi < 0 ) {
				return -1;
			}
			// find to-coordinate
			ti = find( t );
			if( ti < 0 ) {
				return -1;
			}

			{ // ---> initialize
				set_unit(&fconv);
				set_unit(&tconv);
			} // <--- initialize

			// ---> tree traversal loop
			// traverse the parent node of both "t" and "f"
			// until traversal nodes starting from both "t" and "f" is identical
			while(_tree[fi].id != _tree[ti].id){
				// check depth
				int depth = _tree[fi].depth > _tree[ti].depth ? _tree[fi].depth : _tree[ti].depth;

				if( _tree[fi].depth >= depth ){
					// add to coordinate convert matrix
					copy(&ws, &fconv);
					matrix::prod(&_tree[fi], &ws, &fconv);
					// get parent node index
					fi = _tree[fi].parent;
				}

				if( _tree[ti].depth >= depth ){
					// add to coordinate convert matrix
					matrix::copy(&ws, &tconv);
					matrix::prod(&_tree[ti], &ws, &tconv);
					// get parent node index
					ti = _tree[ti].parent;
				}
			} // ---> tree traversal loop

			{ // ---> invert matrix
				matrix::copy(&ws, &tconv);
				matrix::inverse(&ws, &tconv);
			} // <--- invert matrix

			{ // ---> fuse
				matrix::prod(&tconv, &fconv, m);
			} // <--- fuse

			return 0;
		}

		/**
		 * @brief get node id
		 * @param [in] n : name
		 */
		inline int coord_tree::get_node_id(const char *n)
		{
			int i = find(n);
			if( i < 0)	return -1;

			return _tree[i].id;
		}
	}
}

#endif /* YP_COORDINATE_TREE_HPP_ */
