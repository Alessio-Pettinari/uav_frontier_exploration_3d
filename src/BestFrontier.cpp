/*
 * BestFrontier.cpp
 *
 *  Created on: July 14, 2020
 *      Author: Ana Batinovic
 */

#include <uav_frontier_exploration_3d/BestFrontier.h>

namespace best_frontier
{
	BestFrontier::BestFrontier()
	{
		ros::NodeHandle private_nh {ros::NodeHandle("~")};
		
		m_logfile.open("/log_best_frontier.txt");
		m_logfile << "This is a log file for BestFrontier" << endl;

		// Read from yaml file
		private_nh.param("exploration_config_filename", m_configFilename, m_configFilename);
		configureFromFile(m_configFilename);
	}

	bool BestFrontier::configureFromFile(string config_filename)
	{
		cout << "BestFrontier - Configuring sensor specifications from file: " << endl;
		cout << "  " << config_filename << endl;
		
		// Open yaml file with configuration
		YAML::Node config = YAML::LoadFile(config_filename);

		// Get params
		m_resolution = config["octomap"]["resolution"].as<double>();
		m_boxInfGainSize = config["exploration"]["box_length"].as<double>();
		m_kGain = config["exploration"]["k_gain"].as<double>();
		m_lambda = config["exploration"]["lambda"].as<double>();
		m_IsUGV = config["clustering"]["IsUGV"].as<bool>();
		if (config["exploration"]["k_frontier"]){
			m_kFrontier = config["exploration"]["k_frontier"].as<double>();
		} else {
			m_kFrontier = 1.0;
		}

			
		return true;
	}

	point3d BestFrontier::bestFrontierInfGain(
		octomap::OcTree* octree,  point3d currentPosition, KeySet& Cells, point3d ugvFrontier)
	{
		// m_logfile << "bestFrontierInfGain" << endl;
		ros::WallTime startTime_frontier = ros::WallTime::now();
		if (Cells.size() == 0) 
		{
			ROS_WARN("BestFrontier - Zero clustered frontiers!");
			return {};
		}
		vector<pair<point3d, point3d>> candidates;

		for(KeySet::iterator iter = Cells.begin(), 
			end = Cells.end(); iter != end; ++iter)
		{
			// Get cell position
			point3d tempCellPosition = octree->keyToCoord(*iter);
			double x = tempCellPosition.x();
			double y = tempCellPosition.y();
			double z = tempCellPosition.z();
			candidates.push_back(make_pair<point3d, point3d>(point3d(x, y, z), point3d(0.0, 0.0, 0.0)));	
		}

		ROS_INFO_STREAM("Number of candidates: " << candidates.size());
    	ROS_INFO_STREAM("Number of cells: " << Cells.size());
    	ROS_INFO_STREAM("Current position: " << currentPosition);

		// If the cluster point is not in octree
		if (candidates.size() == 0)
		{
			ROS_WARN("BestFrontier - Zero candidates.");	
			return {};
		} 
		// Calculate information gain for every clustered candidate
		std::vector<double> InfGainVector(candidates.size());
	
		for(int i = 0; i < candidates.size(); i++)
		{
			// Get candidate
			auto currCandidate = candidates[i];
			ros::WallTime startTime = ros::WallTime::now();
			double unknownVolume = calcMIBox(octree, currCandidate.first);
			double total_time_1 = (ros::WallTime::now() - startTime).toSec();

			ros::WallTime startTime1 = ros::WallTime::now();
			double tempDistance = calculateDistance(currentPosition, currCandidate.first);
			double total_time_2 = (ros::WallTime::now() - startTime1).toSec();

			
			double distFrontier = 0.0;
			if (!m_IsUGV) { 
				ROS_INFO("UGVFRONTIERAA: -> x: %.3f, y: %.3f, z: %.3f", ugvFrontier.x(), ugvFrontier.y(), ugvFrontier.z());
				distFrontier = calculateDistance(currCandidate.first, ugvFrontier);
			}

    	
			double kGain = m_kGain;
			
			ros::WallTime startTime2 = ros::WallTime::now();
			//InfGainVector[i] = (kGain * unknownVolume * exp(- m_lambda * tempDistance))*(m_kFrontier/(1+distFrontier)); 
			// InfGainVector[i] = (kGain * unknownVolume) + (m_lambda * tempDistance)+ (m_kFrontier/(1+distFrontier));
			// InfGainVector[i] = (kGain * unknownVolume) * (m_lambda * tempDistance)*(m_kFrontier/(1+distFrontier));
			if (m_IsUGV)
			{
				InfGainVector[i] = (kGain * unknownVolume) * (m_lambda * tempDistance);
				//InfGainVector[i] = (0.5* unknownVolume) * (m_lambda * tempDistance);


			} else {
				InfGainVector[i] = (0.5 * unknownVolume) * (0.2 * tempDistance)*(m_kFrontier/(1+distFrontier));
				//InfGainVector[i] = (0.5* unknownVolume) * (0.5 * tempDistance);


			}
 
			double total_time_3 = (ros::WallTime::now() - startTime2).toSec();

			ROS_INFO_STREAM("distFront:" << i << "-" << distFrontier);
			ROS_INFO_STREAM("Candidate " << i << " - InfGain: " << InfGainVector[i]);
			ROS_INFO_STREAM("Distance " << i << " - d:" << tempDistance);
			// ROS_INFO_STREAM("time1:" << total_time_1);
			// ROS_INFO_STREAM("time2:" << total_time_2);
			// ROS_INFO_STREAM("time3:" << total_time_3);
		}

		// Find max element index
		int maxElementIndex = 
			max_element(InfGainVector.begin(), InfGainVector.end()) - InfGainVector.begin();
		ROS_INFO_STREAM("Max element index: " << maxElementIndex);
	
		// Define best frontier
		point3d bestFrontier = point3d(
			candidates[maxElementIndex].first.x(),
			candidates[maxElementIndex].first.y(),
			candidates[maxElementIndex].first.z());
		m_logfile << "Best frontier point: " << bestFrontier << endl;
		ROS_INFO_STREAM("Best frontier point: " << bestFrontier);

		for (int i = 0; i < candidates.size(); i++)
		{
			ROS_INFO_STREAM("Candidate " << i << " - XYZ: " << candidates[i].first);
			ROS_INFO_STREAM("Candidate " << i << " - InfGain: " << InfGainVector[i]);
		}


		double total_time_frontier = (ros::WallTime::now() - startTime_frontier).toSec();
		m_logfile << "Best frontier used total: "<< total_time_frontier << " sec" << endl;
		return bestFrontier;
	}

	double BestFrontier::calcMIBox(const octomap::OcTree *octree, const point3d &sensorOrigin)
	{
		ros::WallTime startTime_frontier = ros::WallTime::now();
		// Calculate number of unchanged cells inside a box around candidate point
		// Propotional to number of the unknown cells inside a box
		
		// Set bounding box
		point3d minPoint, maxPoint;
		double a = m_boxInfGainSize;
		minPoint.x() = sensorOrigin.x() - (a / 2);
		minPoint.y() = sensorOrigin.y() - (a / 2);
		minPoint.z() = sensorOrigin.z() - (a / 2);

		maxPoint.x() = sensorOrigin.x() + (a / 2);
		maxPoint.y() = sensorOrigin.y() + (a / 2);
		maxPoint.z() = sensorOrigin.z() + (a / 2);

		int unknownNum {0};
		int allNum {0};
		for(double ix = minPoint.x(); ix < maxPoint.x(); ix += m_resolution)
		{
			for(double iy = minPoint.y(); iy < maxPoint.y(); iy += m_resolution)
			{
				for (double iz = minPoint.z(); iz < maxPoint.z(); iz += m_resolution)
				{
					allNum++;
					if(!octree->search(ix, iy, iz))
						unknownNum++;
				}
			}
		}	
		return (double)unknownNum / (double)allNum;
	}

}   