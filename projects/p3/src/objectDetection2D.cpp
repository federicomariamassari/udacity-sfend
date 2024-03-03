#include "objectDetection2D.hpp"


using namespace std;


void detectObjects(cv::Mat& img, vector<BoundingBox>& bBoxes, float confThreshold, float nmsThreshold, string basePath, 
  string classesFile, string modelConfiguration, string modelWeights, bool bVis, bool& bExtraAccuracy, 
  bool& bSaveYoloBBFrames, string saveAs)
{
  // Time the YOLOv3 classification process
  auto startTime = chrono::steady_clock::now();

  // Load class names from file
  vector<string> classes;
  ifstream ifs(classesFile.c_str());
  string line;
  while (getline(ifs, line)) classes.push_back(line);
  
  // Load neural network
  cv::dnn::Net net = cv::dnn::readNetFromDarknet(modelConfiguration, modelWeights);
  net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
  net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
  
  // Generate 4D blob from input image
  cv::Mat blob;
  vector<cv::Mat> netOutput;
  double scalefactor = 1 / 255.0;

  // With default size (416 x 416), frames 48-77 will contain an extra bounding box centered on the truck on the RHS
  // and with a few LiDAR points belonging to the preceding vehicle (distorting TTC computation). Slightly increasing
  // the default size to (448 x 448) for those frames resolves the issue, even though at this point both vehicles
  // (ego and preceding) are still, so TTC is unreliable anyway. It is interesting to notice that setting accuracy to
  // a higher size such as (608 x 608) reintroduces the problem, possibly because the truck-centered box also becomes
  // more accurate. Size must be a multiple of 32 (source: Udacity GPT).

  cv::Size size = (bExtraAccuracy) ? cv::Size(448, 448) : cv::Size(416, 416);

  cv::Scalar mean = cv::Scalar(0, 0, 0);
  bool swapRB = false;
  bool crop = false;
  cv::dnn::blobFromImage(img, blob, scalefactor, size, mean, swapRB, crop);
  
  // Get names of output layers
  vector<cv::String> names;
  vector<int> outLayers = net.getUnconnectedOutLayers();  // Get indices of  output layers (with unconnected outputs)
  vector<cv::String> layersNames = net.getLayerNames();  // Get names of all layers in the network
  
  names.resize(outLayers.size());
  for (size_t i = 0; i < outLayers.size(); ++i)  // Get the names of the output layers in names
    names[i] = layersNames[outLayers[i] - 1];
  
  // Invoke forward propagation through network
  net.setInput(blob);
  net.forward(netOutput, names);
  
  // Scan through all bounding boxes and keep only the ones with high confidence
  vector<int> classIds; vector<float> confidences; vector<cv::Rect> boxes;
  for (size_t i = 0; i < netOutput.size(); ++i)
  {
    float* data = (float*) netOutput[i].data;
    for (int j = 0; j < netOutput[i].rows; ++j, data += netOutput[i].cols)
    {
      cv::Mat scores = netOutput[i].row(j).colRange(5, netOutput[i].cols);
      cv::Point classId;
      double confidence;
      
      // Get the value and location of the maximum score
      cv::minMaxLoc(scores, 0, &confidence, 0, &classId);

      if (confidence > confThreshold)
      {
        cv::Rect box; int cx, cy;
        cx = (int) (data[0] * img.cols);
        cy = (int) (data[1] * img.rows);
        box.width = (int) (data[2] * img.cols);
        box.height = (int) (data[3] * img.rows);
        box.x = cx - box.width / 2;  // Left
        box.y = cy - box.height / 2;  // Top
        
        boxes.push_back(box);
        classIds.push_back(classId.x);
        confidences.push_back((float) confidence);
      }
    }
  }

  // Perform non-maxima suppression (NMS)
  vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
  
  for (auto it = indices.begin(); it != indices.end(); ++it)
  {
    BoundingBox bBox;
    bBox.roi = boxes[*it];
    bBox.classID = classIds[*it];
    bBox.confidence = confidences[*it];
    bBox.boxID = (int) bBoxes.size();  // Zero-based unique identifier for this bounding box

    bBoxes.push_back(bBox);
  }

  auto endTime = chrono::steady_clock::now();
  auto elapsedTime = chrono::duration_cast<chrono::microseconds>(endTime - startTime);
  cout << "YOLOv3 classification took: " << elapsedTime.count() / 1000. << " ms" << endl;

  // Show results
  if (bVis) 
  {      
    cv::Mat visImg = img.clone();
    
    for(auto it = bBoxes.begin(); it != bBoxes.end(); ++it)
    {    
      // Draw rectangle displaying the bounding box
      int top, left, width, height;
      top = (*it).roi.y;
      left = (*it).roi.x;
      width = (*it).roi.width;
      height = (*it).roi.height;
      cv::rectangle(visImg, cv::Point(left, top), cv::Point(left + width, top + height),cv::Scalar(0, 255, 0), 2);
      
      string label = cv::format("%.2f", (*it).confidence);
      label = classes[((*it).classID)] + ":" + label;
  
      // Display label at the top of the bounding box
      int baseLine;
      cv::Size labelSize = getTextSize(label, cv::FONT_ITALIC, 0.5, 1, &baseLine);
      top = max(top, labelSize.height);
      
      rectangle(visImg, cv::Point(left, top - round(1.5 * labelSize.height)), 
        cv::Point(left + round(1.5 * labelSize.width), top + baseLine), cv::Scalar(255, 255, 255), cv::FILLED);
      
      cv::putText(visImg, label, cv::Point(left, top), cv::FONT_ITALIC, 0.75, cv::Scalar(0, 0, 0), 1);
    }

    string windowName = "Object classification";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, visImg);

  if (bSaveYoloBBFrames)
  {
    imwrite(saveAs, visImg);
    cout << "Saved image: " << saveAs << endl;
  }

    cv::waitKey(0);  // Wait for key to be pressed
  }
}
