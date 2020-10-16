#include "darknet_ros/YoloObjectDetector.hpp"


namespace darknet_ros{

	std::string YoloObjectDetector::classification_lane(bool left_in, bool right_in){

	  if (left_in && right_in){
	        return "ego_lane";
	  }
	  else if(left_in){
	    return "right_lane";
	  }
	  else if(right_in){
	    return "left_lane";
	  }
	  return "retry";
	}

	float YoloObjectDetector::inter_lane_func(float y,
	              YoloObjectDetector::lane_struct struct_obj){

	  float x = struct_obj.coeff[0] * pow(y,3) +struct_obj.coeff[1] * pow(y,2)
	    + struct_obj.coeff[2] * y + struct_obj.coeff[3];

	  return x;

	}

	float YoloObjectDetector::line_lane_func(float yp, float slope,
	      YoloObjectDetector::lane_struct struct_obj, std::string str)
	{
		if (str=="upper"){
			int size = struct_obj.ys.size();
			float ys = struct_obj.ys[size-1];
			float xs = struct_obj.xs[size-1];
			return slope*(yp-ys) + xs;
		}
		else if(str=="lower"){
			float ys = struct_obj.ys[0];
			float xs = struct_obj.xs[0];
			return slope*(yp-ys) + xs;
		}

		if(str=="upper_topview"){
			int size = struct_obj.ys.size();
			float ys = struct_obj.ys[size-1];
			float xs = struct_obj.xs[size-1];
			Eigen::Vector2f po = topview_conversion(xs, ys, struct_obj.topview_mat);
			return slope*(yp-po(1)) + po(0);
		}
		else if(str=="lower_topview"){
			float ys = struct_obj.ys[0];
			float xs = struct_obj.xs[0];
			Eigen::Vector2f po = topview_conversion(xs, ys, struct_obj.topview_mat);
			return slope*(yp-po(1)) + po(0);
		}
		return -1001;
	}

	Eigen::Vector2f YoloObjectDetector::topview_conversion(float x, float y,
                std::vector<float> topview_mat)
	{

	  Eigen::Vector3f A;
	  Eigen::Matrix3f proj = Eigen::Map<Eigen::Matrix3f>(topview_mat.data());
	  Eigen::Vector3f out;
	  A << x, y, 1;
	  out = proj * A;
	  out(0) = out(0) / out(2);
	  out(1) = out(1) / out(2);

	  Eigen::Vector2f output;
	  output << out(0), out(1);

	  return output;
	}


	float YoloObjectDetector::slope_mean(
                YoloObjectDetector::lane_struct struct_obj, std::string str)
	{

	  if (str=="upper"){

	      std::vector<float> slope;
	      int si = struct_obj.ys.size();
	      int number = 10;
	      if(si<number)
	        number =si - 2;

	      for (int i=0; i<number; i++)
	      {
	        slope.push_back( (float) (struct_obj.xs[si-i-1] - struct_obj.xs[si-i])
	          / (struct_obj.ys[si-i-1] - struct_obj.ys[si-i]));
	      }

	      float slope_ = std::accumulate(slope.begin(), slope.end(), 0.0);
	      slope_ /= number ;
	      slope.clear();

	      return slope_;
	    }
	    else if(str=="lower"){

	    std::vector<float> slope;
	    int si = struct_obj.ys.size();
	    int number = 10;
	    if(si<number)
	      number =si - 2;


	    for (int i=0; i<number; i++)
	    {
	      slope.push_back( (float) (struct_obj.xs[i+1] - struct_obj.xs[i])
	        / (struct_obj.ys[i+1] - struct_obj.ys[i]));
	    }

	    float slope_ = std::accumulate(slope.begin(), slope.end(), 0.0);
	    slope_ /= number ;
	    slope.clear();

	    return slope_;
	  }



	  if (str=="upper_topview"){

	    std::vector<float> slope;
	    int si = struct_obj.ys.size();
	    int number = 10;
	    if(si<10)
	      number =si - 2;

	    for (int i=0; i<number; i++)
	    {
	      Eigen::Vector2f tp_0 = topview_conversion(struct_obj.xs[si-i], struct_obj.ys[si-i],
	              struct_obj.topview_mat);
	      Eigen::Vector2f tp_1 = topview_conversion(struct_obj.xs[si-i-1], struct_obj.ys[si-i-1],
	              struct_obj.topview_mat);

	      slope.push_back( (float) (tp_1(0) - tp_0(0))
	        / (tp_1(1) - tp_0(1)));
	    }

	    float slope_ = std::accumulate(slope.begin(), slope.end(), 0.0);
	    slope_ /= number ;
	    slope.clear();

	    return slope_;
	  }
	  else if(str=="lower_topview"){

	    std::vector<float> slope;
	    int si = struct_obj.ys.size();
	    int number = 10;
	    if(si<10)
	      number =si - 2;


	    for (int i=0; i<number; i++)
	    {
	      Eigen::Vector2f tp_0 = topview_conversion(struct_obj.xs[i], struct_obj.ys[i],
	              struct_obj.topview_mat);
	      Eigen::Vector2f tp_1 = topview_conversion(struct_obj.xs[i+1], struct_obj.ys[i+1],
	              struct_obj.topview_mat);

	      slope.push_back( (float) (tp_1(0) - tp_0(0))
	        / (tp_1(1) - tp_0(1)));
	    }

	    float slope_ = std::accumulate(slope.begin(), slope.end(), 0.0);
	    slope_ /= number ;
	    slope.clear();

	    return slope_;
	  }
	  else{
	    return -1001;
	  }

	}

	std::string YoloObjectDetector::which_lane_is_object_on(int index, float** pre)
	{
		float * xmin = *pre;
		float * xmax = *(pre +1);
		float * ymin = *(pre +2);
		float * ymax = *(pre +3);

		float bmx = (*(xmin+index) + *(xmax+index) ) / 2.;
		float bmy = *(ymax+index);

		float resized_bmx = bmx * 640;
		float resized_bmy = bmy * 400;

		Eigen::Vector2f cen_top = topview_conversion(resized_bmx, resized_bmy, lane_clone_left.topview_mat);
		float topview_cx = cen_top(0);
		float topview_cy = cen_top(1);

		if (lane_clone_left.Class == "left_lane" && lane_clone_right.Class == "right_lane")
		{

		int minimum = lane_clone_left.ys[0];
		int maximum = lane_clone_left.ys[lane_clone_left.ys.size()-1];

		// with interval of lane information
		if ( minimum <= resized_bmy &&  resized_bmy <= maximum){
		  bool left_in = resized_bmx >= inter_lane_func(resized_bmy, lane_clone_left);
		  bool right_in = resized_bmx <= inter_lane_func(resized_bmy, lane_clone_right);

		  std::string final_class = classification_lane(left_in, right_in);
		  if(final_class == "retry"){
		    return "retry, midpoint";
		  }
		  else{
		    return final_class;
		  }
		}
		else if(maximum < resized_bmy){

		  float slope_left = slope_mean(lane_clone_left, "upper");
		  float slope_right = slope_mean(lane_clone_right, "upper");

		  bool left_in = resized_bmx >= line_lane_func(resized_bmy,
		                  slope_left, lane_clone_left,"upper");
		  bool right_in = resized_bmx <= line_lane_func(resized_bmy,
		                  slope_right, lane_clone_right,"upper");

		  std::string final_class = classification_lane(left_in, right_in);
		  if(final_class == "retry"){
		    return "retry, maximum";
		  }
		  else{
		    return final_class;
		  }
		}
		else{

		  // float slope_left = slope_mean(lane_clone_left, "lower_topview");
		  // float slope_right = slope_mean(lane_clone_right, "lower_topview");

		  // bool left_in = topview_cx >= line_lane_func(topview_cy,
		  //                 slope_left,lane_clone_left,"lower_topview");
		  // bool right_in = topview_cx <= line_lane_func(topview_cy,
		  //                 slope_right,lane_clone_right,"lower_topview");
		  float slope_left = slope_mean(lane_clone_left, "lower");
		  float slope_right = slope_mean(lane_clone_right, "lower");
		  bool left_in = resized_bmx >= line_lane_func(resized_bmy,
		                slope_left, lane_clone_left,"lower");
		  bool right_in = resized_bmx <= line_lane_func(resized_bmy,
		                slope_right, lane_clone_right,"lower");
		  std::string final_class = classification_lane(left_in, right_in);


		  if(final_class=="retry"){

		    slope_left = slope_mean(lane_clone_left, "lower_topview");
		    slope_right = slope_mean(lane_clone_right, "lower_topview");

		    left_in = topview_cx >= line_lane_func(topview_cy,
		                    slope_left,lane_clone_left,"lower_topview");
		    right_in = topview_cx <= line_lane_func(topview_cy,
		                    slope_right,lane_clone_right,"lower_topview");
		    final_class = classification_lane(left_in, right_in);
		    // slope_left = slope_mean(lane_clone_left, "lower");
		    // slope_right = slope_mean(lane_clone_right, "lower");
		    // left_in = resized_bmx >= line_lane_func(resized_bmy,
		    //               slope_left, lane_clone_left,"lower");
		    // right_in = resized_bmx <= line_lane_func(resized_bmy,
		    //               slope_right, lane_clone_right,"lower");
		    // final_class = classification_lane(left_in, right_in);
		    return final_class;
		  }
		  else{
		    return final_class;
		  }
		}
		}
			else{
			return "No complete lane";
		}

	}

	void YoloObjectDetector::laneCallback(const lane_msgs::lane_arrayConstPtr& msg){

	  if(lane_activation){
	    for(int i = 0 ; i <2; i++){
	      lane_struct_array[i].xs.clear();
	      lane_struct_array[i].ys.clear();
	      lane_struct_array[i].coeff.clear();
	      lane_struct_array[i].topview_mat.clear();
	      lane_struct_array[i].topview_invmat.clear();
	    }
	  }

	  for(int i = 0; i < msg->lane_array.size(); i++){
	    lane_struct_array[i].Class = msg->lane_array[i].Class;
	    lane_struct_array[i].xs = msg->lane_array[i].xs;
	    lane_struct_array[i].ys = msg->lane_array[i].ys;
	    lane_struct_array[i].coeff = msg->lane_array[i].coeff;
	    lane_struct_array[i].topview_mat.assign(msg->trans_mat.begin(),msg->trans_mat.end());
	    lane_struct_array[i].topview_invmat.assign(msg->inv_mat.begin(),msg->inv_mat.end());
	  }


	  {
	    boost::unique_lock<boost::shared_mutex> locklaneCallback(mutexlaneCallback_);
	    lane_clone_left = lane_struct_array[0];
	    lane_clone_right = lane_struct_array[1];
	    lane_activation = true;
	  }
	}





}

