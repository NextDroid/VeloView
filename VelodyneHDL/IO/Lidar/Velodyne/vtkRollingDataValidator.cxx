// Copyright 2016 Velodyne Lidar, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "vtkRollingDataValidator.h"
#include <algorithm>

vtkRollingDataValidator::vtkRollingDataValidator()
  : vtkRollingDataAccumulator()
{
}

//--------------------------------------------------------------------------------
vtkRollingDataValidator::~vtkRollingDataValidator()
{
}

void vtkRollingDataValidator::setXmlCorrections(const std::vector<HDLLaserCorrection>& xmlCalibration) {

  bool debug = false;

  std::cout << "Setting Xml Calibration Values inside vtkRollingDataAccumulator::setXmlCorrections" << std::endl;

  for (int ii = 0; ii < xmlCalibration.size(); ii++) {

    this->xmlFileCorrections.emplace_back();

    this->xmlFileCorrections.back()["vert_correction"] = xmlCalibration[ii].verticalCorrection;
    this->xmlFileCorrections.back()["rot_correction"] = xmlCalibration[ii].rotationalCorrection;
    this->xmlFileCorrections.back()["dist_correction"] = xmlCalibration[ii].distanceCorrection;
    this->xmlFileCorrections.back()["dist_correction_x"] = xmlCalibration[ii].distanceCorrectionX;
    this->xmlFileCorrections.back()["dist_correction_y"] = xmlCalibration[ii].distanceCorrectionY;
    this->xmlFileCorrections.back()["vert_offset_correction"] = xmlCalibration[ii].verticalOffsetCorrection;
    this->xmlFileCorrections.back()["horiz_offset_correction"] = xmlCalibration[ii].horizontalOffsetCorrection;
    this->xmlFileCorrections.back()["focal_distance"] = xmlCalibration[ii].focalDistance;
    this->xmlFileCorrections.back()["focal_slope"] = xmlCalibration[ii].focalSlope;
    this->xmlFileCorrections.back()["min_intensity"] = xmlCalibration[ii].minIntensity;
    this->xmlFileCorrections.back()["max_intensity"] = xmlCalibration[ii].maxIntensity;

    if (debug) {

      std::cout << "Laser " << ii << ": " 
              << "\n\tRotational Correction: " << this->xmlFileCorrections.back()["vert_correction"]
              << "\n\tVertical Correction: "   << this->xmlFileCorrections.back()["rot_correction"]
              << "\n\tDistance Correction: " << this->xmlFileCorrections.back()["dist_correction"]
              << "\n\tDistance Correction X: " << this->xmlFileCorrections.back()["dist_correction_x"]
              << "\n\tDistance Correction Y: " << this->xmlFileCorrections.back()["dist_correction_y"]
              << "\n\tVertical Offset Correction: " << this->xmlFileCorrections.back()["vert_offset_correction"]
              << "\n\tHorizontal Offset Correction: " << this->xmlFileCorrections.back()["horiz_offset_correction"]
              << "\n\tFocal Distance: " << this->xmlFileCorrections.back()["focal_distance"]
              << "\n\tFocal Slope: " << this->xmlFileCorrections.back()["focal_slope"]
              << "\n\tLaser 0 Min Intensity: " << this->xmlFileCorrections.back()["min_intensity"]
              << "\n\tMax Intensity: " << this->xmlFileCorrections.back()["max_intensity"]
              << std::endl;
    } 
  }

  if (debug) {
    exit(10);
  }
}

void vtkRollingDataValidator::findBeginPositionsComparable() {

  for (int ii = 0; ii < accumulatedDataComparable.size()-5; ii++) {
    auto pairU = accumulatedDataComparable[ii+0];
    auto pairN = accumulatedDataComparable[ii+1];
    auto pairI = accumulatedDataComparable[ii+2];
    auto pairT = accumulatedDataComparable[ii+3];
    auto pairHash = accumulatedDataComparable[ii+4];

    if ( (pairU.dataType == '1' && pairU.dataValue == 'U') &&
         (pairN.dataType == '2' && pairN.dataValue == 'N') &&
         (pairI.dataType == '3' && pairI.dataValue == 'I') &&
         (pairT.dataType == '4' && pairT.dataValue == 'T') &&
         (pairHash.dataType == '5' && pairHash.dataValue == '#') ) 
    {
      beginPositionsComparable.emplace_back(ii);
    }
  }
}

std::tuple<int, std::vector<int>> vtkRollingDataValidator::getCycleNumberAndIdxs(const int laser_id, const std::string &field) {

  int baseCycleNumber = 4*laser_id + 2;
  int cycleOffset = 0;
  std::vector<int> idxs;

  if (field == "vert_correction") {
    cycleOffset = 0;
    idxs.insert(idxs.end(), {2, 3});
  } else if (field == "rot_correction") {
    cycleOffset = 0;
    idxs.insert(idxs.end(), {4, 5}); 
  } else if (field == "dist_correction") {
    cycleOffset = 0;
    idxs.insert(idxs.end(), {6, 7});
  } else if (field == "dist_correction_x") {
    cycleOffset = 1;
    idxs.insert(idxs.end(), {1, 2});
  } else if (field == "dist_correction_y") {
    cycleOffset = 1;
    idxs.insert(idxs.end(), {3, 4});
  } else if (field == "vert_offset_correction"){
    cycleOffset = 1;
    idxs.insert(idxs.end(), {5, 6});
  } else if (field == "horiz_offset_correction") {
    cycleOffset = 1;
    idxs.insert(idxs.end(), {7, 8});
  } else if (field == "focal_distance") {
    cycleOffset = 2;
    idxs.insert(idxs.end(), {2, 3});
  } else if (field == "focal_slope") {
    cycleOffset = 2;
    idxs.insert(idxs.end(), {4, 5});
  } else if (field == "min_intensity") {
    cycleOffset = 2;
    idxs.insert(idxs.end(), {6});
  } else if (field == "max_intensity") {
    cycleOffset = 2;
    idxs.insert(idxs.end(), {7});
  } else {
    std::cout << "Did not use an existing calibration field in vtkRollingDataValidator::getCycleNumberAndIdxs() -- " << field << std::endl;
  }

  int pos = baseCycleNumber + cycleOffset;
  return std::make_tuple(pos, idxs);
}


double vtkRollingDataValidator::getFieldValueScale(const std::string &field) {
  if (field == "vert_correction") {
    return 1.0/100.0;
  }
  else if (field == "rot_correction"){
    return 1.0/100.0;
  }
  else if (field == "dist_correction"){
    return 1.0/10.0/100.0;  // NOTE: Extra factor of 100.0 is to reverse correction done in vtkVelodyneHDLReader::vtkInternal::LoadCorrectionsFile
  }
  else if (field == "dist_correction_x"){
    return 1.0/10.0/100.0;  // NOTE: Extra factor of 100.0 is to reverse correction done in vtkVelodyneHDLReader::vtkInternal::LoadCorrectionsFile
  }
  else if (field == "dist_correction_y"){
    return 1.0/10.0/100.0;  // NOTE: Extra factor of 100.0 is to reverse correction done in vtkVelodyneHDLReader::vtkInternal::LoadCorrectionsFile
  }
  else if (field == "vert_offset_correction"){
    return 1.0/10.0/100.0;  // NOTE: Extra factor of 100.0 is to reverse correction done in vtkVelodyneHDLReader::vtkInternal::LoadCorrectionsFile
  }
  else if (field == "horiz_offset_correction"){
    return 1.0/10.0/100.0;  // NOTE: Extra factor of 100.0 is to reverse correction done in vtkVelodyneHDLReader::vtkInternal::LoadCorrectionsFile
  }
  else if (field == "focal_distance"){
    return 1.0/10.0/100.0;  // NOTE: Extra factor of 100.0 is to reverse correction done in vtkVelodyneHDLReader::vtkInternal::LoadCorrectionsFile
  }
  else if (field == "focal_slope"){
    return 1.0/10.0/100.0;  // NOTE: Extra factor of 100.0 is to reverse correction done in vtkVelodyneHDLReader::vtkInternal::LoadCorrectionsFile
  }
  else if (field == "min_intensity"){
    return 1.0;
  }
  else if (field == "max_intensity"){
    return 1.0;
  } else {
    std::cout << "Did not use an existing calibration field in vtkRollingDataValidator::getFieldValueScale() -- " << field << std::endl;
    return 1.0;
  }
}

double vtkRollingDataValidator::getThresholdForField(const std::string &field) {
  if (field == "vert_correction") {
    return 1.0/100.0;
  }
  else if (field == "rot_correction"){
    return 1.0/100.0;
  }
  else if (field == "dist_correction"){
    return 1.0/10.0/100.0;  // NOTE: Extra factor of 100.0 is to reverse correction done in vtkVelodyneHDLReader::vtkInternal::LoadCorrectionsFile
  }
  else if (field == "dist_correction_x"){
    return 1.0/10.0/100.0;  // NOTE: Extra factor of 100.0 is to reverse correction done in vtkVelodyneHDLReader::vtkInternal::LoadCorrectionsFile
  }
  else if (field == "dist_correction_y"){
    return 1.0/10.0/100.0;  // NOTE: Extra factor of 100.0 is to reverse correction done in vtkVelodyneHDLReader::vtkInternal::LoadCorrectionsFile
  }
  else if (field == "vert_offset_correction"){
    return 1.0/10.0/100.0;  // NOTE: Extra factor of 100.0 is to reverse correction done in vtkVelodyneHDLReader::vtkInternal::LoadCorrectionsFile
  }
  else if (field == "horiz_offset_correction"){
    return 1.0/10.0/100.0;  // NOTE: Extra factor of 100.0 is to reverse correction done in vtkVelodyneHDLReader::vtkInternal::LoadCorrectionsFile
  }
  else if (field == "focal_distance"){
    return 1.0/10.0/100.0;  // NOTE: Extra factor of 100.0 is to reverse correction done in vtkVelodyneHDLReader::vtkInternal::LoadCorrectionsFile
  }
  else if (field == "focal_slope"){
    return 1.0/10.0/100.0;  // NOTE: Extra factor of 100.0 is to reverse correction done in vtkVelodyneHDLReader::vtkInternal::LoadCorrectionsFile
  }
  else if (field == "min_intensity"){
    return 1.0;
  }
  else if (field == "max_intensity"){
    return 1.0;
  } else {
    std::cout << "Did not use an existing calibration field in vtkRollingDataValidator::getThresholdForField() -- " << field << std::endl;
    return 1.0;
  }
}


double vtkRollingDataValidator::resolveStreamValue(const std::vector<TypeValueDataPair> &streamDataCycle, const int laser_id, const std::string &field) {

  // Get Cycle Number and Idx:
  auto cycleAndIdxs = getCycleNumberAndIdxs(laser_id, field);
  auto cycleNum = std::get<0>(cycleAndIdxs);
  auto idxs = std::get<1>(cycleAndIdxs);

  // Get Stream Bytes:
  std::vector<unsigned char> streamValues;
  for (const auto idx : idxs) {
    int streamPos = (cycleNum-1)*7 + idx - 1; 
    streamValues.emplace_back(streamDataCycle[streamPos].dataValue);
  }

  // Convert Bytes to Int:
  int16_t val;
  if (streamValues.size() == 2) {
    val = (streamValues[1]  << 8) + streamValues[0];
  } else if (streamValues.size() == 1) {
    val = streamValues[0];
  } else {
    std::cout << "Unexpected number of bytes parsed out stream data in vtkRollingDataValidator::resolveStreamValue() -- number of bytes: " << streamValues.size() << std::endl;
  }

  // Convert to double and scale:
  double scale = getFieldValueScale(field);
  double scaledValue = scale * val;

  // Return scaled value:
  return scaledValue;
}


bool vtkRollingDataValidator::verifyCalibrationData(bool checkIncompleteCycles) {

  // Find full cycles in accumulatedDataComparable:
  this->findBeginPositionsComparable();


  std::vector<long> cycleLengths;
  long firstFullCycle = -1;
  for (int ii = 0; ii < beginPositionsComparable.size(); ii++) {
    
    long nextBeginPosition;
    if (ii == beginPositionsComparable.size()-1) {
      nextBeginPosition = accumulatedDataComparable.size();
    } else {
      nextBeginPosition = beginPositionsComparable[ii+1];
    }   

    cycleLengths.emplace_back(nextBeginPosition - beginPositionsComparable[ii]);

    if (cycleLengths.back() == expectedLengthComparable && firstFullCycle == -1) {
      firstFullCycle = beginPositionsComparable[ii];
    }
  }
  bool fullCycleExists = (firstFullCycle != -1);

  // Initialize Bingo Array (called bingo because we want to verify/fill every laser calibration value)
  int numLasers = xmlFileCorrections.size();
  std::vector<bool> bingo(numLasers*fields.size()); // 64 lasers with set of fields

  // Full Cycle Exists:
  if (fullCycleExists) {
    // Approach: Check every calibration value from xml file against stream data

    // Get full cycle:
    auto startIter = accumulatedDataComparable.begin()+firstFullCycle;
    auto endIter = accumulatedDataComparable.begin()+firstFullCycle + expectedLengthComparable;
    std::vector<TypeValueDataPair> streamDataCycle(startIter, endIter);

    // Compare each laser calibration field to corresponding xml data:
    int bingoIdx = 0;
    for (int laser_id = 0; laser_id < numLasers; laser_id++) {
      // std::cout << "Laser ID: " << laser_id << std::endl;
      for (const auto field : fields) {
        
        // Get XML Value:
        auto xmlValue = xmlFileCorrections[laser_id][field];
        
        // Resolve Stream Data Value:
        auto streamValue = resolveStreamValue(streamDataCycle, laser_id, field);

        // Get Threshold for comparing values:
        auto threshold = 2.0*getThresholdForField(field);

        // Compare XML value to Stream value:
        bool withinThreshold = fabs(xmlValue - streamValue) < threshold;

        // std::cout << "\tField: " << field 
        //           << "\n\t\tXML Value: " << xmlValue 
        //           << "\n\t\tStream Value: " << streamValue
        //           << "\n\t\tThreshold: " << threshold
        //           << "\n\t\tWithin Threshold: " << withinThreshold
        //           << std::endl;

        bingo[bingoIdx] = withinThreshold;
        bingoIdx++;
      }
    }

    // Determine Number of Lasers within Threshold:
    int numWithinThreshold = std::count(bingo.begin(), bingo.end(), true);
    int totalNumber = bingo.size();

    if (numWithinThreshold == totalNumber) {
      std::cout << "Validated XML File against Streamed Calibration Data -- Passed [num matched: " << numWithinThreshold << "/" << totalNumber << "]" << std::endl;
      return true;
    } else {
      std::cout << "Could not validate XML File against Streamed Calibration Data -- Failed [num failed: " << (totalNumber - numWithinThreshold) << "/" << totalNumber << "]" << std::endl;
      return false;
    }

  } else if (!fullCycleExists && checkIncompleteCycles) {
    // Approach: Find longest consecutive chain of validated xml values against stream calibration values
    int maxNumWithinThreshold = 0;

    for (int ii = 0; ii < beginPositionsComparable.size(); ii++) {
      auto startIdx = beginPositionsComparable[ii];
      auto cycleLength = cycleLengths[ii];

      // Get Cycle:
      auto startIter = accumulatedDataComparable.begin()+startIdx;
      auto endIter = accumulatedDataComparable.begin()+firstFullCycle + cycleLength;
      std::vector<TypeValueDataPair> streamDataCycle(startIter, endIter);

      int bingoIdx = 0;
      bool lastPairWasMatch = true;
      for (int laser_id = 0; laser_id < numLasers; laser_id++) {
        for (const auto field : fields) {
          if (bingoIdx < cycleLength) {
            if (lastPairWasMatch) {

              // Get XML Value: 
              auto xmlValue = xmlFileCorrections[laser_id][field];

              // Get Stream Data Value: 
              auto streamValue = resolveStreamValue(streamDataCycle, laser_id, field);

              // Get Threshold for comparing values:
              auto threshold = 2.0*getThresholdForField(field);

              // Compare XML value to Stream value:
              bool withinThreshold = fabs(xmlValue - streamValue) < threshold;


              bingo[bingoIdx] = bingoIdx || withinThreshold;
              lastPairWasMatch = withinThreshold;
              bingoIdx++;

            } else if (!lastPairWasMatch) {
              // TODO add some logic to handle case when last pair wasn't a match
              continue;
            }
          }
        } // for field
      } // for laser_id

      int numWithinThreshold = bingoIdx;
      maxNumWithinThreshold = std::count(bingo.begin(), bingo.end(), true);
      int totalNumber = bingo.size();
      std::cout << "Current cycle's number of matches: " << numWithinThreshold << "\tMost number of consecutive matches found so far: " << maxNumWithinThreshold << " [out of: " << totalNumber << "]" << std::endl; 

    }// for ii (each cycle)

    // TODO compute packet loss rate

    // double packetLossRate = 

    int requiredThreshold = 10;
    std::cout << "Finished Validation" << std::endl; 
    if (maxNumWithinThreshold > requiredThreshold) {
      std::cout << "At least " << requiredThreshold << " consecutive matches were found -- greater than [UNKNOWN] percent confidence that the xml file matches the stream calibration data" << std::endl;
      std::cout << "Largest consecutive chain of matches: " << maxNumWithinThreshold << std::endl;
      return true; 
    } else {
      std::cout << "Found less than " << requiredThreshold << " consecutive matches -- could not sufficiently validate xml file against stream data" << std::endl;
      std::cout << "Largest consecutive chain of matches: " << maxNumWithinThreshold << std::endl;
      return false;
    }
  }

  return false;
}

