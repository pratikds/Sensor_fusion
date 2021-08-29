# SFND 3D Object Tracking

Welcome to the final project of the camera course. By completing all the lessons, you now have a solid understanding of keypoint detectors, descriptors, and methods to match them between successive images. Also, you know how to detect objects in an image using the YOLO deep-learning framework. And finally, you know how to associate regions in a camera image with Lidar points in 3D space. Let's take a look at our program schematic to see what we already have accomplished and what's still missing.

<img src="images/course_code_structure.png" width="779" height="414" />

In this final project, you will implement the missing parts in the schematic. To do this, you will complete four major tasks: 
1. First, you will develop a way to match 3D objects over time by using keypoint correspondences. 
2. Second, you will compute the TTC based on Lidar measurements. 
3. You will then proceed to do the same using the camera, which requires to first associate keypoint matches to regions of interest and then to compute the TTC based on those matches. 
4. And lastly, you will conduct various tests with the framework. Your goal is to identify the most suitable detector/descriptor combination for TTC estimation and also to search for problems that can lead to faulty measurements by the camera or Lidar sensor. In the last course of this Nanodegree, you will learn about the Kalman filter, which is a great way to combine the two independent TTC measurements into an improved version which is much more reliable than a single sensor alone can be. But before we think about such things, let us focus on your final project in the camera course. 

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.

//All the required steps are shown below as the tasks with the code

 //TASK FP.1 -> Developed a way to match 3D objects over time by using keypoint correspondences using the following code. It was done by Matching list of 3D objects (vector<BoundingBox>) between current and previous frame (implement ->matchBoundingBoxes)
  
  void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    int prev_box_size = prevFrame.boundingBoxes.size();
    int curr_box_size = currFrame.boundingBoxes.size();
    int prev_curr_box_score[prev_box_size][curr_box_size] = {};

    // iterate pnt matchs, cnt box-box match score
    for (auto it = matches.begin(); it != matches.end() - 1; it++)
    {
        // prev pnt
        cv::KeyPoint prev_key_pnt = prevFrame.keypoints[it->queryIdx];
        cv::Point prev_pnt = cv::Point(prev_key_pnt.pt.x, prev_key_pnt.pt.y);

        // curr pnt
        cv::KeyPoint curr_key_pnt = currFrame.keypoints[it->trainIdx];
        cv::Point curr_pnt = cv::Point(curr_key_pnt.pt.x, curr_key_pnt.pt.y);

        // get corresponding box with the point
        std::vector<int> prev_box_id_list, curr_box_id_list;
        for (int i = 0; i < prev_box_size; ++i)
        {
            if (prevFrame.boundingBoxes[i].roi.contains(prev_pnt))
            {
                prev_box_id_list.push_back(i);
            }
        }
        for (int j = 0; j < curr_box_size; ++j)
        {
            if (currFrame.boundingBoxes[j].roi.contains(curr_pnt))
            {
                curr_box_id_list.push_back(j);
            }
        }

        // add cnt to prev_curr_box_score
        for (int i: prev_box_id_list)
        {
            for (int j: curr_box_id_list)
            {
                prev_curr_box_score[i][j] += 1;
            }
        }
    } // END OF THE PNT MATCH

    // for each box in prevFrame, find the box with highest score in currFrame
    for (int i = 0; i < prev_box_size; ++i) 
    {
        int max_score = 0;
        int best_idx = 0;
        
        for (int j = 0; j < curr_box_size; ++j)
        {
            if (prev_curr_box_score[i][j] > max_score)
            {
                max_score = prev_curr_box_score[i][j];
                best_idx = j;
            }
        }

        bbBestMatches[i] = best_idx;
    }   
   
}
  
  
  
 //// TASK FP.2 -> Computed the TTC based on Lidar measurement, Lidar data (implement -> computeTTCLidar) as the code below.
 
void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
   // auxiliary variables
    double dT = 1.0 / frameRate; // time between two measurements in seconds

    // find closest distance to Lidar points 
    double minXPrev = 1e9, minXCurr = 1e9;

    vector<double> prev_vector;
    for(auto it=lidarPointsPrev.begin(); it!=lidarPointsPrev.end(); ++it) {
        prev_vector.push_back(it->x);
    }
    sort(prev_vector.begin(), prev_vector.end());
    minXPrev = prev_vector[prev_vector.size() * 1 / 5];

    vector<double> curr_vector;
    for(auto it=lidarPointsCurr.begin(); it!=lidarPointsCurr.end(); ++it) {
        curr_vector.push_back(it->x);
    }
    sort(curr_vector.begin(), curr_vector.end());
    minXCurr = curr_vector[curr_vector.size() * 1 / 5];

    // compute TTC from both measurements
    TTC = minXCurr * dT / (minXPrev-minXCurr);

    cout << "lidar ttc cal------------------" << endl;
    cout << "minXPrev: " << minXPrev << endl;
    cout << "minXCurr: " << minXCurr << endl;
    cout << "-------------------------------" << endl;
 
  
}
  
 //// TASK FP.3 -> Assigned enclosed keypoint matches to bounding box (implement -> clusterKptMatchesWithROI) as the code Below.

void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
   double  distance_mean_val = 0.0;
   double size = 0.0;
   for (auto it = kptMatches.begin();  it!=kptMatches.end(); ++it)
   {
       
      cv::KeyPoint curr_pnt = kptsCurr[it->trainIdx];
        cv::KeyPoint prev_pnt = kptsPrev[it->queryIdx];

        if (boundingBox.roi.contains(curr_pnt.pt))
        {
            distance_mean_val += cv::norm(curr_pnt.pt - prev_pnt.pt);
            size += 1;
        }
   }
   distance_mean_val = distance_mean_val / size;
   // filter point match based on point match distance
   for (auto it = kptMatches.begin(); it != kptMatches.end(); ++it)
    {
        cv::KeyPoint curr_pnt = kptsCurr[it->trainIdx];
        cv::KeyPoint prev_pnt = kptsPrev[it->queryIdx];

        if (boundingBox.roi.contains(curr_pnt.pt))
        {
            double curr_dist = cv::norm(curr_pnt.pt - prev_pnt.pt);

            if (curr_dist < distance_mean_val * 1.3)
            {
                boundingBox.keypoints.push_back(curr_pnt);
                boundingBox.kptMatches.push_back(*it);
            }
        }
   }    
}
  
 
 //// TASK FP.4 -> In the Previous Step, we did the same using the camera like lidar, which is like first associating the  keypoint matches to regions of interest and then computed the TTC based on those matches (implement -> computeTTCCamera) using the code below.

  // Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    
    // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer kpt. loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner kpt.-loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    // STUDENT TASK (replacement for meanDistRatio)
    std::sort(distRatios.begin(), distRatios.end());
    long medIndex = floor(distRatios.size() / 2.0);
    double medDistRatio = distRatios.size() % 2 == 0 ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0 : distRatios[medIndex]; // compute median dist. ratio to remove outlier influence

    double dT = 1 / frameRate;
    TTC = -dT / (1 - medDistRatio);
    // EOF STUDENT TASK
  
  
}

 
///Task 5 (Performance Evaluation 1) --> have conducted tests with the final project code, especially with regard to the Lidar part. Have Looked for several examples where I had an impression that the Lidar-based TTC estimate is way off. So my observations are as follows.
  
  In the 3 Continous frames, the lidar bawed TTC suddenly rises from 12.52 sec to 31 sec and again drops to 14 sec. The ego car moves very much slowsly between this frames and only covered distance of 0.06meters, but small distance caused even bigger fluctuation but where as camera TTC is stable between previous and current frame as shown from the observations below.
  
Lidar : AKAZE+BRISK : FAST + BRISK : SHITOMASI + ORB

12.52 :    14.76    :  12.53      :   11.65
31.45 :    13.45    :  14.36      :   12.06
14.46 :    14.25    :  12.60      :   13.07
  
  
  // Task 6 ( Performance Evaluation 2)--> Conducted various tests with the framework,to identify the most suitable detector/descriptor combination for TTC estimation and searched for problems that can lead to faulty measurements by the camera or Lidar sensor.
  
--> In several detector / descriptor combinations and the look is given at the differences in TTC estimation.
  
I use the best detector/descriptor Combination  in terms of its relative performance and is also used in mid term project.
  
AKAZE + BRISK ,FAST + BRISK ,SHITOMASI + ORB  to compare against lidar TTC estimate

Below is the list explains TTC Comparision of Libar and other camera based Techniques in seconds with different dectector/descriptor combination.
  
Lidar : AKAZE+BRISK : FAST + BRISK : SHITOMASI + ORB

  12.52 :    14.76    :  12.53      :   11.65
  31.45 :    13.45    :  14.36      :   12.06
  14.46 :    14.25    :  12.60      :   13.07
  10.17 :    14.12    :  34.70      :   12.09 
  13.96 :    15.08    :  12.43      :   13.22
  11.35 :    16.20    :  18.92      :   12.85
  14.82 :    14.39    :  11.30      :   11.88
  13.16 :    13.96    :  13.20      :   11.24
  15.21 :    11.47    :  12.52      :   13.94
  11.92 :    12.65    :  14.25      :   11.47
  9.62  :    11.29    :  11.40      :   11.56
  8.93  :    10.44    :  12.24      :   11.63
  9.54  :    10.54    :  12.13      :   11.42
  7.68  :    10.00    :  12.08      :   10.55
  9.20  :    9.99     :  12.17      :   8.89
  11.75 :    9.44     :  8.51       :   9.53
  10.40 :    8.98     :  11.54      :   8.21
   
  
  From the measurement as shown above, it is clearly visible that Lidar based TTC iis way off than Camera Based TTC and Lidar based TTC is much more stable. Though the several Detector/Descriptor combinations have been evaluated, it seems all the methods calculates their own TTC estimation and they are heavily fluctuating. Even out of 3 evaluated combination  of methods of camera based TTC, SHITOMASI+ORB is little more comparable to Lidarbased TTC as compared to others.