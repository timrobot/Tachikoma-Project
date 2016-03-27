#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/ml.hpp>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <njson/json.hpp>

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;
using namespace cv::flann;
using namespace cv::ml;
using json = nlohmann::json;

int main(int argc, char *argv[]) {
  if (argc < 4) {
    printf("Usage: %s %s %s %s\n", argv[0], "input_file.json", "output_dictionary.yml", "output_descriptor.yml");
    return 1;
  }

  // open input json file
  cout << "open input json file" << endl;
  string params, temp;
  ifstream params_file(argv[1]);
  while (getline(params_file, temp)) {
    params += temp;
  }
  params_file.close();
  json config = json::parse(params);

  // create feature extractor
  cout << "create feature extractor" << endl;
  Ptr<SIFT> detector = SIFT::create();
  vector<KeyPoint> keypoints;
  Mat description;
  Mat features;

  // read in the images
  cout << "read in the images" << endl;
  vector<string> tags;
  vector<Mat> training_images;
  vector<string> training_tags;
  for (auto &_tag : config["train_images"]) {
    string tag = _tag;
    tags.push_back(tag);
    for (auto &_image_name : config[tag]) {
      string image_name = _image_name;
      training_images.push_back(imread(image_name));
      training_tags.push_back(tag);
    }
  }

  // use the feature extractor on the images
  cout << "use the feature extractor on the images" << endl;
  for (Mat &image : training_images) {
    Mat gray;
    cvtColor(image, gray, COLOR_BGR2GRAY);
    detector->detect(gray, keypoints);
    detector->compute(gray, keypoints, description);
    features.push_back(description);
  }

  // once we have the features, then we can create a bag of words from them
  cout << "once we have the features, then we can create a bag of words from them" << endl;
  int dictionary_size = 200;
  TermCriteria tc(CV_TERMCRIT_ITER, 100, 0.001);
  BOWKMeansTrainer bowTrainer(dictionary_size, tc, 1, KMEANS_PP_CENTERS);
  Mat dictionary = bowTrainer.cluster(features);
  FileStorage fs_dictionary(argv[2], FileStorage::WRITE);
  fs_dictionary << "vocabulary" << dictionary;
  fs_dictionary.release();

  // now that we have the bag of words, lets create a vocabulary
  cout << "now that we have the bag of words, lets create a vocabulary" << endl;
  Ptr<DescriptorMatcher> matcher = makePtr<FlannBasedMatcher>(
      makePtr<KDTreeIndexParams>(5), makePtr<SearchParams>(50));
  Ptr<DescriptorExtractor> extractor = detector;
  BOWImgDescriptorExtractor bowde(extractor, matcher);
  bowde.setVocabulary(dictionary);

  // generate samples from the bow extractor
  cout << "generate samples from the bow extractor" << endl;
  Mat samples;
  Mat responses;
  Mat bowDescriptor;
  FileStorage fs_descriptor(argv[3], FileStorage::WRITE);
  for (int k = 0; k < training_images.size(); k++) {
    Mat &image = training_images[k];
    detector->detect(image, keypoints);
    bowde.compute(image, keypoints, bowDescriptor);
    samples.push_back(bowDescriptor);
    string &tag = training_tags[k];
    for (int i = 0; i < tags.size(); i++) {
      if (tags[i].compare(tag) == 0) {
        responses.push_back(i + 1); // this is the id of the item
        break;
      }
    }
    string json_tag = tag + to_string(k);
    fs_descriptor << json_tag << bowDescriptor;
  }
  fs_descriptor.release();

  // train the SVM
  cout << "train the SVM" << endl;
  Ptr<SVM> classifier = SVM::create();
  classifier->train(samples, ROW_SAMPLE, responses);

  // for now do nothing with it
  return 0;
}
