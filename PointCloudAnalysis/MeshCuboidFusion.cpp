#include "MeshCuboidFusion.h"

#include "MeshCuboidParameters.h"
#include "ICP.h"
#include "Utilities.h"

#include <Eigen/Core>
#include <MRFEnergy.h>


MeshCuboidVoxelGrid::MeshCuboidVoxelGrid(MyMesh::Point _min, MyMesh::Point _max, Real _unit_size)
	: min_(_min)
	, max_(_max)
{
	MyMesh::Normal diff = max_ - min_;
	for (unsigned int i = 0; i < 3; ++i)
	{
		assert(diff[i] > 0);
		n_voxels_[i] = static_cast<int>(std::round(diff[i] / _unit_size));
		if (n_voxels_[i] < 2)
		{
			n_voxels_[i] = 2;
			Real center = 0.5 * (min_[i] + max_[i]);
			min_[i] = center - _unit_size;
			max_[i] = center + _unit_size;
		}
	}
}

MeshCuboidVoxelGrid::~MeshCuboidVoxelGrid()
{
}

int MeshCuboidVoxelGrid::n_voxels() const
{
	return (n_voxels_[0] * n_voxels_[1] * n_voxels_[2]);
}

int MeshCuboidVoxelGrid::n_axis_voxels(const int _axis_index) const
{
	assert(_axis_index < 3);
	return n_voxels_[_axis_index];
}

int MeshCuboidVoxelGrid::get_voxel_index(MeshCuboidVoxelIndex3D _xyz_index) const
{
	assert(_xyz_index[0] >= 0);
	assert(_xyz_index[1] >= 0);
	assert(_xyz_index[2] >= 0);
	assert(_xyz_index[0] < n_voxels_[0]);
	assert(_xyz_index[1] < n_voxels_[1]);
	assert(_xyz_index[2] < n_voxels_[2]);

	int voxel_index =
		_xyz_index[0] * (n_voxels_[1] * n_voxels_[2])
		+ _xyz_index[1] * n_voxels_[2]
		+ _xyz_index[2];

	assert(voxel_index >= 0);
	assert(voxel_index < n_voxels());

	// DEBUG.
	MeshCuboidVoxelIndex3D test_xyz_index = get_voxel_index(voxel_index);
	assert(_xyz_index[0] == test_xyz_index[0]);
	assert(_xyz_index[1] == test_xyz_index[1]);
	assert(_xyz_index[2] == test_xyz_index[2]);

	return voxel_index;
}

MeshCuboidVoxelIndex3D MeshCuboidVoxelGrid::get_voxel_index(const int _voxel_index) const
{
	assert(_voxel_index >= 0);
	assert(_voxel_index < n_voxels());
	MeshCuboidVoxelIndex3D xyz_index;

	int temp = _voxel_index;
	xyz_index[0] = temp / (n_voxels_[1] * n_voxels_[2]);
	temp -= (xyz_index[0] * n_voxels_[1] * n_voxels_[2]);

	xyz_index[1] = temp / n_voxels_[2];
	temp -= (xyz_index[1] * n_voxels_[2]);

	xyz_index[2] = temp;

	assert(xyz_index[0] >= 0);
	assert(xyz_index[1] >= 0);
	assert(xyz_index[2] >= 0);
	assert(xyz_index[0] < n_voxels_[0]);
	assert(xyz_index[1] < n_voxels_[1]);
	assert(xyz_index[2] < n_voxels_[2]);
	return xyz_index;
}

int MeshCuboidVoxelGrid::get_voxel_index(const MyMesh::Point _point) const
{
	MyMesh::Normal diff = max_ - min_;
	MeshCuboidVoxelIndex3D xyz_index;

	for (unsigned int i = 0; i < 3; ++i)
	{
		assert(diff[i] > 0);
		if (_point[i] < min_[i] || _point[i] > max_[i])
		{
			// Out of voxel grid range.
			return -1;
		}
		else
		{
			xyz_index[i] = static_cast<int>((_point[i] - min_[i]) / diff[i] * n_voxels_[i]);
			xyz_index[i] = std::max(xyz_index[i], 0);
			xyz_index[i] = std::min(xyz_index[i], n_voxels_[i] - 1);
		}
	}

	return get_voxel_index(xyz_index);
}

MyMesh::Point MeshCuboidVoxelGrid::get_center(const int _voxel_index) const
{
	MeshCuboidVoxelIndex3D xyz_index = get_voxel_index(_voxel_index);
	MyMesh::Normal diff = max_ - min_;
	MyMesh::Point center;

	for (unsigned int i = 0; i < 3; ++i)
	{
		assert(diff[i] > 0);
		Real ratio = static_cast<Real>(xyz_index[i] + 0.5) / n_voxels_[i];
		center[i] = min_[i] + ratio * diff[i];
	}

	return center;
}

void MeshCuboidVoxelGrid::get_centers(std::vector<MyMesh::Point> &_centers) const
{
	_centers.clear();
	_centers.resize(n_voxels());
	for (int voxel_index = 0; voxel_index < n_voxels(); ++voxel_index)
		_centers[voxel_index] = get_center(voxel_index);
}

void MeshCuboidVoxelGrid::get_point_correspondences(
	const std::vector<MyMesh::Point> &_points,
	std::vector<int> &_points_to_voxels,
	std::vector< std::list<int> > &_voxels_to_points) const
{
	const unsigned int num_points = _points.size();
	_points_to_voxels.clear();
	_voxels_to_points.clear();
	_points_to_voxels.resize(num_points);
	_voxels_to_points.resize(n_voxels());

	for (unsigned int point_index = 0; point_index < num_points; ++point_index)
	{
		int voxel_index = get_voxel_index(_points[point_index]);

		// Out of voxel grid range.
		if (voxel_index < 0)
			continue;

		assert(voxel_index < n_voxels());
		_points_to_voxels[point_index] = voxel_index;
		_voxels_to_points[voxel_index].push_back(point_index);
	}
}

void MeshCuboidVoxelGrid::get_voxel_occupancies(const std::vector<MyMesh::Point> &_points,
	Eigen::VectorXd &_voxel_occupancies) const
{
	const unsigned int num_points = _points.size();
	_voxel_occupancies = Eigen::VectorXd::Zero(n_voxels());

	for (unsigned int point_index = 0; point_index < num_points; ++point_index)
	{
		int voxel_index = get_voxel_index(_points[point_index]);

		// Out of voxel grid range.
		if (voxel_index < 0)
			continue;

		assert(voxel_index < n_voxels());
		_voxel_occupancies[voxel_index] = 1;
	}
}

void MeshCuboidVoxelGrid::get_distance_map(ANNpointArray &_ann_points, ANNkd_tree *__ann_kd_tree,
	Eigen::VectorXd &_voxel_to_point_distances) const
{
	std::vector<MyMesh::Point> center_points;
	get_centers(center_points);
	assert(center_points.size() == n_voxels());

	Eigen::MatrixXd center_points_mat(3, center_points.size());
	for (unsigned int voxel_index = 0; voxel_index < center_points.size(); ++voxel_index)
	{
		for (int i = 0; i < 3; ++i)
			center_points_mat.col(voxel_index)[i] = center_points[voxel_index][i];
	}

	ICP::get_closest_points(__ann_kd_tree, center_points_mat, _voxel_to_point_distances);
}

void run_part_ICP(MeshCuboidStructure &_input, const MeshCuboidStructure &_ground_truth)
{
	const Real neighbor_distance = FLAGS_param_sparse_neighbor_distance
		* _ground_truth.mesh_->get_object_diameter();

	unsigned int num_labels = _ground_truth.num_labels();
	for (LabelIndex label_index = 0; label_index < num_labels; ++label_index)
	{
		if (_ground_truth.label_cuboids_[label_index].empty()
			|| _input.label_cuboids_[label_index].empty())
			continue;

		MeshCuboid *ground_truth_cuboid = _ground_truth.label_cuboids_[label_index].front();
		MeshCuboid *input_cuboid = _input.label_cuboids_[label_index].front();
		assert(ground_truth_cuboid);
		assert(input_cuboid);

		if (ground_truth_cuboid->num_sample_points() == 0
			|| input_cuboid->num_sample_points() == 0)
			continue;

		Eigen::MatrixXd ground_truth_sample_points(3, ground_truth_cuboid->num_sample_points());
		Eigen::MatrixXd input_sample_points(3, input_cuboid->num_sample_points());

		for (SamplePointIndex sample_point_index = 0; sample_point_index < ground_truth_cuboid->num_sample_points();
			++sample_point_index)
		{
			MeshSamplePoint *sample_point = ground_truth_cuboid->get_sample_point(sample_point_index);
			assert(sample_point);
			MyMesh::Point point = ground_truth_cuboid->get_sample_point(sample_point_index)->point_;
			for (unsigned int i = 0; i < 3; ++i)
				ground_truth_sample_points.col(sample_point_index)(i) = point[i];

			//
			//is_ground_truth_point_visited[sample_point->sample_point_index_] = true;
			//
		}

		for (SamplePointIndex sample_point_index = 0; sample_point_index < input_cuboid->num_sample_points();
			++sample_point_index)
		{
			MeshSamplePoint *sample_point = input_cuboid->get_sample_point(sample_point_index);
			assert(sample_point);
			MyMesh::Point point = input_cuboid->get_sample_point(sample_point_index)->point_;
			for (unsigned int i = 0; i < 3; ++i)
				input_sample_points.col(sample_point_index)(i) = point[i];
		}

		Eigen::Matrix3d rotation_mat;
		Eigen::Vector3d translation_vec;
		double icp_error = ICP::run_iterative_closest_points(input_sample_points, ground_truth_sample_points,
			rotation_mat, translation_vec, &neighbor_distance);

		if (translation_vec.norm() > 2 * neighbor_distance)
			icp_error = -1;

		// If ICP failed, recover the original input point cloud.
		if (icp_error < 0)
		{
			for (SamplePointIndex sample_point_index = 0; sample_point_index < input_cuboid->num_sample_points();
				++sample_point_index)
			{
				MeshSamplePoint *sample_point = input_cuboid->get_sample_point(sample_point_index);
				assert(sample_point);
				MyMesh::Point point = input_cuboid->get_sample_point(sample_point_index)->point_;
				for (unsigned int i = 0; i < 3; ++i)
					input_sample_points.col(sample_point_index)(i) = point[i];
			}
		}


		for (SamplePointIndex sample_point_index = 0; sample_point_index < input_cuboid->num_sample_points();
			++sample_point_index)
		{
			MeshSamplePoint* input_sample_point = input_cuboid->get_sample_point(sample_point_index);
			assert(input_sample_point);

			for (unsigned int i = 0; i < 3; ++i)
				input_sample_point->point_[i] = input_sample_points.col(sample_point_index)[i];
		}
	}
}

void create_voxel_grid(
	const MeshCuboid *_symmetry_cuboid,
	const MeshCuboid *_database_cuboid,
	MyMesh::Point &_bbox_min, MyMesh::Point &_bbox_max)
{
	Eigen::MatrixXd symmetry_points, database_points;
	_symmetry_cuboid->get_sample_points(symmetry_points);
	_database_cuboid->get_sample_points(database_points);

	Eigen::Vector3d symmetry_bbox_min_vec = symmetry_points.rowwise().minCoeff();
	Eigen::Vector3d symmetry_bbox_max_vec = symmetry_points.rowwise().maxCoeff();
	Eigen::Vector3d database_bbox_min_vec = database_points.rowwise().minCoeff();
	Eigen::Vector3d database_bbox_max_vec = database_points.rowwise().maxCoeff();

	for (unsigned int i = 0; i < 3; ++i)
	{
		_bbox_min[i] = std::min(symmetry_bbox_min_vec[i], database_bbox_min_vec[i]);
		_bbox_max[i] = std::max(symmetry_bbox_max_vec[i], database_bbox_max_vec[i]);
	}
}

void get_smoothed_voxel_visibility(
	const MeshCuboidVoxelGrid &_voxels,
	const MeshCuboid *_ground_truth_cuboid,
	const double *_occlusion_modelview_matrix,
	const MeshCuboidStructure &_original_cuboid_structure,
	const Real &_occlusion_radius,
	const Real &_smoothing_parameter,
	std::vector<Real> &_voxel_visibility)
{
	// Smoothing using MRF.
	MRFEnergy<TypeBinary>* mrf;
	MRFEnergy<TypeBinary>::NodeId* nodes;
	MRFEnergy<TypeBinary>::Options options;
	TypeBinary::REAL energy, lowerBound;

	const int nodeNum = _voxels.n_voxels(); // number of nodes
	mrf = new MRFEnergy<TypeBinary>(TypeBinary::GlobalSize());
	nodes = new MRFEnergy<TypeBinary>::NodeId[nodeNum];

	// construct energy
	for (unsigned int voxel_index = 0; voxel_index < _voxels.n_voxels(); ++voxel_index)
	{
		nodes[voxel_index] = mrf->AddNode(TypeBinary::LocalSize(), TypeBinary::NodeData(
			_voxel_visibility[voxel_index], 1.0 - _voxel_visibility[voxel_index]));
	}

	for (unsigned int voxel_index = 0; voxel_index < _voxels.n_voxels(); ++voxel_index)
	{
		MeshCuboidVoxelIndex3D xyz_index = _voxels.get_voxel_index(voxel_index);

		for (unsigned int axis_index = 0; axis_index < 3; ++axis_index)
		{
			MeshCuboidVoxelIndex3D n_xyz_index = xyz_index;
			if (n_xyz_index[axis_index] + 1 == _voxels.n_axis_voxels(axis_index))
				continue;

			++n_xyz_index[axis_index];
			unsigned int n_voxel_index = _voxels.get_voxel_index(n_xyz_index);
			assert(voxel_index < n_voxel_index);
			assert(n_voxel_index < _voxels.n_voxels());
			mrf->AddEdge(nodes[voxel_index], nodes[n_voxel_index], TypeBinary::EdgeData(
				0, _smoothing_parameter, _smoothing_parameter, 0));
		}
	}

	/////////////////////// TRW-S algorithm //////////////////////
	options.m_iterMax = 30; // maximum number of iterations
	mrf->Minimize_TRW_S(options, lowerBound, energy);

	// read solution
	_voxel_visibility.resize(_voxels.n_voxels(), 0.0);
	for (unsigned int voxel_index = 0; voxel_index < _voxels.n_voxels(); ++voxel_index)
	{
		_voxel_visibility[voxel_index] =
			static_cast<Real>(mrf->GetSolution(nodes[voxel_index]));
	}

	delete nodes;
	delete mrf;
}

void merge_symmetric_cuboids_visibility(
	const MeshCuboidSymmetryGroup *_symmetry_group,
	const MeshCuboidVoxelGrid &_voxels_1,
	const MeshCuboidVoxelGrid &_voxels_2,
	std::vector<Real> &_voxel_visibility_1,
	std::vector<Real> &_voxel_visibility_2)
{
	assert(_symmetry_group);

	const int num_voxels_1 = _voxels_1.n_voxels();
	assert(_voxel_visibility_1.size() == num_voxels_1);

	for (int voxel_index_1 = 0; voxel_index_1 < num_voxels_1; ++voxel_index_1)
	{
		MyMesh::Point center_1 = _voxels_1.get_center(voxel_index_1);

		for (unsigned int symmetry_order = 1; symmetry_order < _symmetry_group->num_symmetry_orders(); ++symmetry_order)
		{
			assert(symmetry_order > 0);

			MyMesh::Point center_2 = _symmetry_group->get_symmetric_point(center_1, symmetry_order);
			int voxel_index_2 = _voxels_2.get_voxel_index(center_2);
			if (voxel_index_2 < 0 || voxel_index_2 >= num_voxels_1)
				continue;

			// A voxel is visible if at least one of voxels in symmetric cuboids is visible.
			Real visibility = std::max(_voxel_visibility_1[voxel_index_1], _voxel_visibility_2[voxel_index_2]);
			_voxel_visibility_1[voxel_index_1] = visibility;
			_voxel_visibility_2[voxel_index_2] = visibility;
		}
	}
}

void fill_voxels_using_visibility(
	const MeshCuboidVoxelGrid &_voxels,
	const std::vector<Real> &_voxel_visibility_values,
	const MeshCuboid *_symmetry_cuboid,
	const MeshCuboid *_database_cuboid,
	MeshCuboidStructure &_output_cuboid_structure,
	MeshCuboid *_output_cuboid)
{
	std::vector<MyMesh::Point> symmetry_points;
	std::vector<int> symmetry_points_to_voxels;
	std::vector< std::list<int> > symmetry_voxels_to_points;

	_symmetry_cuboid->get_sample_points(symmetry_points);
	_voxels.get_point_correspondences(symmetry_points,
		symmetry_points_to_voxels, symmetry_voxels_to_points);

	std::vector<MyMesh::Point> database_points;
	std::vector<int> database_points_to_voxels;
	std::vector< std::list<int> > database_voxels_to_points;

	_database_cuboid->get_sample_points(database_points);
	_voxels.get_point_correspondences(database_points,
		database_points_to_voxels, database_voxels_to_points);

	for (unsigned int voxel_index = 0; voxel_index < _voxels.n_voxels(); ++voxel_index)
	{
		assert(voxel_index < symmetry_voxels_to_points.size());
		assert(voxel_index < database_voxels_to_points.size());
		MeshCuboidVoxelIndex3D xyz_index = _voxels.get_voxel_index(voxel_index);

		const std::list<int>& symmetry_voxel_point_indices = symmetry_voxels_to_points[voxel_index];
		const std::list<int>& database_voxel_point_indices = database_voxels_to_points[voxel_index];

		if (_voxel_visibility_values[voxel_index] > 0.5)
		{
			for (std::list<int>::const_iterator it = symmetry_voxel_point_indices.begin();
				it != symmetry_voxel_point_indices.end(); ++it)
			{
				MeshSamplePoint *sample_point = _symmetry_cuboid->get_sample_point(*it);
				assert(sample_point);

				MeshSamplePoint *new_sample_point = _output_cuboid_structure.add_sample_point(
					sample_point->point_, sample_point->normal_);
				_output_cuboid->add_sample_point(new_sample_point);
			}
		}
		else
		{
			for (std::list<int>::const_iterator it = database_voxel_point_indices.begin();
				it != database_voxel_point_indices.end(); ++it)
			{
				MeshSamplePoint *sample_point = _database_cuboid->get_sample_point(*it);
				assert(sample_point);

				MeshSamplePoint *new_sample_point = _output_cuboid_structure.add_sample_point(
					sample_point->point_, sample_point->normal_);
				_output_cuboid->add_sample_point(new_sample_point);
			}
		}
	}
}

void reconstruct_fusion_simple(
	const MeshCuboidStructure &_symmetry_cuboid_structure,
	const MeshCuboidStructure &_database_cuboid_structure,
	MeshCuboidStructure &_output_cuboid_structure)
{
	assert(_symmetry_cuboid_structure.num_labels() == _database_cuboid_structure.num_labels());

	const Real neighbor_distance = FLAGS_param_sparse_neighbor_distance
		* _output_cuboid_structure.mesh_->get_object_diameter();

	_output_cuboid_structure = _symmetry_cuboid_structure;
	unsigned int num_labels = _symmetry_cuboid_structure.num_labels();

	for (LabelIndex label_index = 0; label_index < num_labels; ++label_index)
	{
		if (_symmetry_cuboid_structure.label_cuboids_[label_index].empty()
			|| _database_cuboid_structure.label_cuboids_[label_index].empty()
			|| _output_cuboid_structure.label_cuboids_[label_index].empty())
			continue;

		MeshCuboid *symmetry_cuboid = _symmetry_cuboid_structure.label_cuboids_[label_index].front();
		MeshCuboid *database_cuboid = _database_cuboid_structure.label_cuboids_[label_index].front();
		MeshCuboid *output_cuboid = _output_cuboid_structure.label_cuboids_[label_index].front();
		assert(symmetry_cuboid);
		assert(database_cuboid);
		assert(output_cuboid);

		if (symmetry_cuboid->num_sample_points() == 0
			|| database_cuboid->num_sample_points() == 0)
			continue;


		Eigen::MatrixXd symmetry_sample_points(3, symmetry_cuboid->num_sample_points());
		Eigen::MatrixXd database_sample_points(3, database_cuboid->num_sample_points());

		for (SamplePointIndex sample_point_index = 0; sample_point_index < symmetry_cuboid->num_sample_points();
			++sample_point_index)
		{
			assert(symmetry_cuboid->get_sample_point(sample_point_index));
			MyMesh::Point point = symmetry_cuboid->get_sample_point(sample_point_index)->point_;
			for (unsigned int i = 0; i < 3; ++i)
				symmetry_sample_points.col(sample_point_index)(i) = point[i];
		}

		for (SamplePointIndex sample_point_index = 0; sample_point_index < database_cuboid->num_sample_points();
			++sample_point_index)
		{
			assert(database_cuboid->get_sample_point(sample_point_index));
			MyMesh::Point point = database_cuboid->get_sample_point(sample_point_index)->point_;
			for (unsigned int i = 0; i < 3; ++i)
				database_sample_points.col(sample_point_index)(i) = point[i];
		}

		Eigen::Matrix3d rotation_mat;
		Eigen::Vector3d translation_vec;
		double icp_error = ICP::run_iterative_closest_points(database_sample_points, symmetry_sample_points,
			rotation_mat, translation_vec, &neighbor_distance);
		//std::cout << "ICP Error = " << icp_error << std::endl;

		for (SamplePointIndex sample_point_index = 0; sample_point_index < database_cuboid->num_sample_points();
			++sample_point_index)
		{
			MeshSamplePoint *sample_point = database_cuboid->get_sample_point(sample_point_index);
			assert(sample_point);

			MyMesh::Point new_point;
			for (unsigned int i = 0; i < 3; ++i)
				new_point[i] = database_sample_points.col(sample_point_index)[i];

			MeshSamplePoint *new_sample_point = _output_cuboid_structure.add_sample_point(
				new_point, sample_point->normal_);
			output_cuboid->add_sample_point(new_sample_point);
		}
	}

	/*
	ret = cuboid_structure_.load_dense_sample_points(dense_sample_filepath.c_str());
	assert(ret);
	set_modelview_matrix(_occlusion_modelview_matrix, false);
	remove_occluded_points();
	set_modelview_matrix(_snapshot_modelview_matrix);

	cuboid_structure_.copy_sample_points_to_symmetric_position();

	std::vector<LabelIndex> reconstructed_label_indices;
	reconstructed_label_indices.push_back(0);
	reconstructed_label_indices.push_back(1);
	reconstruct_using_database(&reconstructed_label_indices);
	*/
}

bool get_fusion_cuboids(const LabelIndex _label_index,
	const MeshCuboidStructure &_symmetry_cuboid_structure,
	const MeshCuboidStructure &_database_cuboid_structure,
	const MeshCuboidStructure &_output_cuboid_structure,
	MeshCuboid *&_symmetry_cuboid, MeshCuboid *&_database_cuboid, MeshCuboid *&_output_cuboid)
{
	if (_symmetry_cuboid_structure.label_cuboids_[_label_index].empty()
		|| _database_cuboid_structure.label_cuboids_[_label_index].empty()
		|| _output_cuboid_structure.label_cuboids_[_label_index].empty())
	{
		_symmetry_cuboid = NULL;
		_database_cuboid = NULL;
		_output_cuboid = NULL;
		return false;
	}

	assert(_symmetry_cuboid_structure.label_cuboids_[_label_index].size() <= 1);
	assert(_database_cuboid_structure.label_cuboids_[_label_index].size() <= 1);
	assert(_output_cuboid_structure.label_cuboids_[_label_index].size() <= 1);

	_symmetry_cuboid = _symmetry_cuboid_structure.label_cuboids_[_label_index].front();
	_database_cuboid = _database_cuboid_structure.label_cuboids_[_label_index].front();
	_output_cuboid = _output_cuboid_structure.label_cuboids_[_label_index].front();

	assert(_symmetry_cuboid);
	assert(_database_cuboid);
	assert(_output_cuboid);

	if (_symmetry_cuboid->num_sample_points() == 0) _symmetry_cuboid = NULL;
	if (_database_cuboid->num_sample_points() == 0) _database_cuboid = NULL;
	return (_symmetry_cuboid && _database_cuboid);
}

void mark_cuboid_sample_points(
	const MeshCuboid *_cuboid,
	const MeshCuboidStructure &_cuboid_structure,
	bool *_is_sample_point_visited)
{
	assert(_cuboid);
	for (SamplePointIndex sample_point_index = 0; sample_point_index < _cuboid->num_sample_points();
		++sample_point_index)
	{
		MeshSamplePoint *sample_point = _cuboid->get_sample_point(sample_point_index);
		assert(sample_point);
		assert(sample_point->sample_point_index_ < _cuboid_structure.num_sample_points());
		_is_sample_point_visited[sample_point->sample_point_index_] = true;
	}
}

void copy_sample_points(const MeshCuboid *_from_cuboid,
	MeshCuboidStructure &_to_cuboid_structure, MeshCuboid *_to_cuboid)
{
	assert(_from_cuboid);
	assert(_to_cuboid);

	for (unsigned int i = 0; i < _from_cuboid->num_sample_points(); ++i)
	{
		MeshSamplePoint *sample_point = _from_cuboid->get_sample_point(i);
		assert(sample_point);

		MeshSamplePoint *new_sample_point = _to_cuboid_structure.add_sample_point(
			sample_point->point_, sample_point->normal_);
		_to_cuboid->add_sample_point(new_sample_point);
	}
}

void fusing_single_symmetry_cuboid(LabelIndex _label_index,
	const MeshCuboidSymmetryGroup *_symmetry_group,
	const double *_occlusion_modelview_matrix,
	const MeshCuboidStructure &_original_cuboid_structure,
	const MeshCuboidStructure &_symmetry_cuboid_structure,
	const MeshCuboidStructure &_database_cuboid_structure,
	MeshCuboidStructure &_output_cuboid_structure,
	bool *_is_symmetry_point_visited,
	bool *_is_label_index_visited)
{
	assert(_occlusion_modelview_matrix);
	assert(_is_symmetry_point_visited);
	assert(_is_label_index_visited);

	const Real occlusion_radius = FLAGS_param_fusion_grid_size;
	const Real visibility_smoothing_prior = FLAGS_param_fusion_visibility_smoothing_prior;

	MeshCuboid *symmetry_cuboid = NULL, *database_cuboid = NULL, *output_cuboid = NULL;
	bool ret = get_fusion_cuboids(_label_index,
		_symmetry_cuboid_structure, _database_cuboid_structure, _output_cuboid_structure,
		symmetry_cuboid, database_cuboid, output_cuboid);

	if (!ret)
	{
		if (output_cuboid)
		{
			if (symmetry_cuboid)
				copy_sample_points(symmetry_cuboid, _output_cuboid_structure, output_cuboid);
			else if (database_cuboid)
				copy_sample_points(database_cuboid, _output_cuboid_structure, output_cuboid);
		}
		return;
	}

	_is_label_index_visited[_label_index] = true;
	std::cout << "Single symmetry: (" << _label_index << ")" << std::endl;

	// Mark visited symmetry sample points.
	mark_cuboid_sample_points(symmetry_cuboid, _symmetry_cuboid_structure, _is_symmetry_point_visited);

	// Define local coordinates voxel grid.
	MyMesh::Point bbox_min, bbox_max;
	create_voxel_grid(symmetry_cuboid, database_cuboid, bbox_min, bbox_max);
	MeshCuboidVoxelGrid voxels(bbox_min, bbox_max, occlusion_radius);
	std::vector<MyMesh::Point> voxel_centers;
	voxels.get_centers(voxel_centers);


	std::cout << "Computing visibility values... ";

	std::vector<Real> voxel_visibility;
	MeshCuboid::compute_cuboid_surface_point_visibility(
		_occlusion_modelview_matrix, occlusion_radius, _original_cuboid_structure.sample_points_,
		voxel_centers, NULL, voxel_visibility);

	std::cout << "Computing cuboid surface point visibility done." << std::endl;

	// Merge visibility values for voxels in symmetric cuboids.
	merge_symmetric_cuboids_visibility(_symmetry_group, voxels, voxels, voxel_visibility, voxel_visibility);
	
	std::cout << "Merge symmetric cuboids visibility done." << std::endl;

	// Smoothing.
	get_smoothed_voxel_visibility(
		voxels, symmetry_cuboid, _occlusion_modelview_matrix, _original_cuboid_structure,
		occlusion_radius, visibility_smoothing_prior, voxel_visibility);

	std::cout << "Done." << std::endl;


	std::cout << "Filling voxels... ";
	fill_voxels_using_visibility(
		voxels, voxel_visibility, symmetry_cuboid, database_cuboid,
		_output_cuboid_structure, output_cuboid);
	std::cout << "Done." << std::endl;
}

void fusing_pair_symmetry_cuboids(LabelIndex _label_index_1, LabelIndex _label_index_2,
	const MeshCuboidSymmetryGroup *_symmetry_group,
	const double *_occlusion_modelview_matrix,
	const MeshCuboidStructure &_original_cuboid_structure,
	const MeshCuboidStructure &_symmetry_cuboid_structure,
	const MeshCuboidStructure &_database_cuboid_structure,
	MeshCuboidStructure &_output_cuboid_structure,
	bool *_is_symmetry_point_visited,
	bool *_is_label_index_visited)
{
	assert(_occlusion_modelview_matrix);
	assert(_is_symmetry_point_visited);
	assert(_is_label_index_visited);

	const Real occlusion_radius = FLAGS_param_fusion_grid_size;
	const Real visibility_smoothing_prior = FLAGS_param_fusion_visibility_smoothing_prior;


	MeshCuboid *symmetry_cuboid_1 = NULL, *database_cuboid_1 = NULL, *output_cuboid_1 = NULL;
	bool ret_1 = get_fusion_cuboids(_label_index_1,
		_symmetry_cuboid_structure, _database_cuboid_structure, _output_cuboid_structure,
		symmetry_cuboid_1, database_cuboid_1, output_cuboid_1);

	MeshCuboid *symmetry_cuboid_2 = NULL, *database_cuboid_2 = NULL, *output_cuboid_2 = NULL;
	bool ret_2 = get_fusion_cuboids(_label_index_2,
		_symmetry_cuboid_structure, _database_cuboid_structure, _output_cuboid_structure,
		symmetry_cuboid_2, database_cuboid_2, output_cuboid_2);

	if (!ret_1 || !ret_2)
	{
		if (output_cuboid_1 && output_cuboid_2)
		{
			if (symmetry_cuboid_1 && symmetry_cuboid_2)
			{
				copy_sample_points(symmetry_cuboid_1, _output_cuboid_structure, output_cuboid_1);
				copy_sample_points(symmetry_cuboid_2, _output_cuboid_structure, output_cuboid_2);
			}
			else if (database_cuboid_1 && database_cuboid_2)
			{
				copy_sample_points(database_cuboid_1, _output_cuboid_structure, output_cuboid_1);
				copy_sample_points(database_cuboid_2, _output_cuboid_structure, output_cuboid_2);
			}
		}
		return;
	}

	_is_label_index_visited[_label_index_1] = true;
	_is_label_index_visited[_label_index_2] = true;
	std::cout << "Pair symmetry: (" << _label_index_1 << ", " << _label_index_2 << ")" << std::endl;

	// Mark visited symmetry sample points.
	mark_cuboid_sample_points(symmetry_cuboid_1, _symmetry_cuboid_structure, _is_symmetry_point_visited);
	mark_cuboid_sample_points(symmetry_cuboid_2, _symmetry_cuboid_structure, _is_symmetry_point_visited);

	// Define local coordinates voxel grid.
	MyMesh::Point bbox_min_1, bbox_max_1;
	create_voxel_grid(symmetry_cuboid_1, database_cuboid_1, bbox_min_1, bbox_max_1);
	MeshCuboidVoxelGrid voxels_1(bbox_min_1, bbox_max_1, occlusion_radius);
	std::vector<MyMesh::Point> voxel_centers_1;
	voxels_1.get_centers(voxel_centers_1);

	MyMesh::Point bbox_min_2, bbox_max_2;
	create_voxel_grid(symmetry_cuboid_2, database_cuboid_2, bbox_min_2, bbox_max_2);
	MeshCuboidVoxelGrid voxels_2(bbox_min_2, bbox_max_2, occlusion_radius);
	std::vector<MyMesh::Point> voxel_centers_2;
	voxels_2.get_centers(voxel_centers_2);


	std::cout << "Computing visibility values... ";

	std::vector<Real> voxel_visibility_1;
	MeshCuboid::compute_cuboid_surface_point_visibility(
		_occlusion_modelview_matrix, occlusion_radius, _original_cuboid_structure.sample_points_,
		voxel_centers_1, NULL, voxel_visibility_1);

	std::vector<Real> voxel_visibility_2;
	MeshCuboid::compute_cuboid_surface_point_visibility(
		_occlusion_modelview_matrix, occlusion_radius, _original_cuboid_structure.sample_points_,
		voxel_centers_2, NULL, voxel_visibility_2);

	// Merge visibility values for voxels in symmetric cuboids.
	merge_symmetric_cuboids_visibility(_symmetry_group, voxels_1, voxels_2, voxel_visibility_1, voxel_visibility_2);
	merge_symmetric_cuboids_visibility(_symmetry_group, voxels_2, voxels_1, voxel_visibility_2, voxel_visibility_1);
	

	// Smoothing.
	get_smoothed_voxel_visibility(
		voxels_1, symmetry_cuboid_1, _occlusion_modelview_matrix, _original_cuboid_structure,
		occlusion_radius, visibility_smoothing_prior, voxel_visibility_1);
	get_smoothed_voxel_visibility(
		voxels_2, symmetry_cuboid_2, _occlusion_modelview_matrix, _original_cuboid_structure,
		occlusion_radius, visibility_smoothing_prior, voxel_visibility_2);

	std::cout << "Done." << std::endl;


	std::cout << "Filling voxels... ";
	fill_voxels_using_visibility(
		voxels_1, voxel_visibility_1, symmetry_cuboid_1, database_cuboid_1, _output_cuboid_structure, output_cuboid_1);
	fill_voxels_using_visibility(
		voxels_2, voxel_visibility_2, symmetry_cuboid_2, database_cuboid_2, _output_cuboid_structure, output_cuboid_2);
	std::cout << "Done." << std::endl;
}

void reconstruct_fusion(const char *_mesh_filepath,
	const double *_snapshot_modelview_matrix,
	const double *_occlusion_modelview_matrix,
	const MeshCuboidStructure &_original_cuboid_structure,
	const MeshCuboidStructure &_symmetry_cuboid_structure,
	const MeshCuboidStructure &_database_cuboid_structure,
	MeshCuboidStructure &_output_cuboid_structure,
	bool _add_outliers)
{
	assert(_symmetry_cuboid_structure.num_labels() == _database_cuboid_structure.num_labels());

	const Real occlusion_radius = FLAGS_param_fusion_grid_size;
	const Real visibility_smoothing_prior = FLAGS_param_fusion_visibility_smoothing_prior;


	_output_cuboid_structure.clear_sample_points();


	std::cout << "Computing ICP for each part... ";
	MeshCuboidStructure aligned_database_cuboid_structure = _database_cuboid_structure;
	run_part_ICP(aligned_database_cuboid_structure, _symmetry_cuboid_structure);
	std::cout << "Done." << std::endl;


	// Check whether a sample point in the symmetry reconstruction is visited or not.
	bool *is_symmetry_point_visited = new bool[_symmetry_cuboid_structure.num_sample_points()];
	memset(is_symmetry_point_visited, false, _symmetry_cuboid_structure.num_sample_points() * sizeof(bool));

	bool *is_label_index_visited = new bool[_symmetry_cuboid_structure.num_labels()];
	memset(is_label_index_visited, false, _symmetry_cuboid_structure.num_labels() * sizeof(bool));


	for (std::vector< MeshCuboidReflectionSymmetryGroup* >::const_iterator it = _symmetry_cuboid_structure.reflection_symmetry_groups_.begin();
		it != _symmetry_cuboid_structure.reflection_symmetry_groups_.end(); ++it)
	{
		MeshCuboidReflectionSymmetryGroup* symmetry_group = (*it);
		assert(symmetry_group);
		MeshCuboidSymmetryGroupInfo symmetry_group_info = symmetry_group->get_symmetry_group_info();

		// Single symmetric cuboids.
		for (std::vector<LabelIndex>::const_iterator jt = symmetry_group_info.single_label_indices_.begin();
			jt != symmetry_group_info.single_label_indices_.end(); ++jt)
		{
			LabelIndex label_index = (*jt);
			fusing_single_symmetry_cuboid(label_index, symmetry_group,
				_occlusion_modelview_matrix, _original_cuboid_structure,
				_symmetry_cuboid_structure, aligned_database_cuboid_structure, _output_cuboid_structure,
				is_symmetry_point_visited, is_label_index_visited);
		}

		// Pair symmetric cuboids.
		for (std::vector< std::pair<LabelIndex, LabelIndex> >::const_iterator jt = symmetry_group_info.pair_label_indices_.begin();
			jt != symmetry_group_info.pair_label_indices_.end(); ++jt)
		{
			LabelIndex label_index_1 = (*jt).first;
			LabelIndex label_index_2 = (*jt).second;
			fusing_pair_symmetry_cuboids(label_index_1, label_index_2, symmetry_group,
				_occlusion_modelview_matrix, _original_cuboid_structure,
				_symmetry_cuboid_structure, aligned_database_cuboid_structure, _output_cuboid_structure,
				is_symmetry_point_visited, is_label_index_visited);
		}
	}

	for (std::vector< MeshCuboidRotationSymmetryGroup* >::const_iterator it = _symmetry_cuboid_structure.rotation_symmetry_groups_.begin();
		it != _symmetry_cuboid_structure.rotation_symmetry_groups_.end(); ++it)
	{
		MeshCuboidRotationSymmetryGroup* symmetry_group = (*it);
		assert(symmetry_group);
		MeshCuboidSymmetryGroupInfo symmetry_group_info = symmetry_group->get_symmetry_group_info();

		// Single symmetric cuboids.
		for (std::vector<LabelIndex>::const_iterator jt = symmetry_group_info.single_label_indices_.begin();
			jt != symmetry_group_info.single_label_indices_.end(); ++jt)
		{
			LabelIndex label_index = (*jt);
			fusing_single_symmetry_cuboid(label_index, symmetry_group,
				_occlusion_modelview_matrix, _original_cuboid_structure,
				_symmetry_cuboid_structure, aligned_database_cuboid_structure, _output_cuboid_structure,
				is_symmetry_point_visited, is_label_index_visited);
		}

		// NOTE:
		// Pairwise rotational symmetry is not considered.
	}


	// Other single cuboids.
	unsigned int num_labels = _symmetry_cuboid_structure.num_labels();
	for (LabelIndex label_index = 0; label_index < num_labels; ++label_index)
	{
		if (is_label_index_visited[label_index])
			continue;

		MeshCuboid *symmetry_cuboid, *database_cuboid, *output_cuboid;
		bool ret = get_fusion_cuboids(label_index,
			_symmetry_cuboid_structure, aligned_database_cuboid_structure, _output_cuboid_structure,
			symmetry_cuboid, database_cuboid, output_cuboid);
		if (!ret) continue;

		is_label_index_visited[label_index] = true;
		std::cout << "Asymmetry: (" << label_index << ")" << std::endl;

		// Mark visited symmetry sample points.
		mark_cuboid_sample_points(symmetry_cuboid, _symmetry_cuboid_structure, is_symmetry_point_visited);

		// Define local coordinates voxel grid.
		MyMesh::Point bbox_min, bbox_max;
		create_voxel_grid(symmetry_cuboid, database_cuboid,
			bbox_min, bbox_max);
		MeshCuboidVoxelGrid voxels(bbox_min, bbox_max, occlusion_radius);
		std::vector<MyMesh::Point> voxel_centers;
		voxels.get_centers(voxel_centers);

		std::cout << "Computing visibility values... ";
		std::vector<Real> voxel_visibility;
		MeshCuboid::compute_cuboid_surface_point_visibility(
			_occlusion_modelview_matrix, occlusion_radius, _original_cuboid_structure.sample_points_,
			voxel_centers, NULL, voxel_visibility);

		get_smoothed_voxel_visibility(
			voxels, symmetry_cuboid, _occlusion_modelview_matrix, _original_cuboid_structure,
			occlusion_radius, visibility_smoothing_prior, voxel_visibility);
		std::cout << "Done." << std::endl;

		std::cout << "Filling voxels... ";
		fill_voxels_using_visibility(
			voxels, voxel_visibility, symmetry_cuboid, database_cuboid, _output_cuboid_structure, output_cuboid);
		std::cout << "Done." << std::endl;
	}


	// Add unvisited (unsegmented) sample points in symmetry reconstruction.
	for (SamplePointIndex sample_point_index = 0; sample_point_index < _symmetry_cuboid_structure.num_sample_points();
		++sample_point_index)
	{
		if (!is_symmetry_point_visited[sample_point_index])
		{
			const MeshSamplePoint *sample_point = _symmetry_cuboid_structure.sample_points_[sample_point_index];
			assert(sample_point);
			MeshSamplePoint *new_sample_point = _output_cuboid_structure.add_sample_point(
				sample_point->point_, sample_point->normal_);
		}
	}

	delete[] is_symmetry_point_visited;
	delete[] is_label_index_visited;
}