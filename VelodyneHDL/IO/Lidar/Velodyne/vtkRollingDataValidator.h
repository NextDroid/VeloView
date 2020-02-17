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
#include "vtkRollingDataAccumulator.h"

#ifndef VTKROLLINGDATAVALIDATOR_H
#define VTKROLLINGDATAVALIDATOR_H

class VTK_EXPORT vtkRollingDataValidator : public vtkRollingDataAccumulator
{
public:
  void appendData(unsigned int timestamp, unsigned char dataType, unsigned char dataValue);
  void setXmlCorrections(const std::vector<HDLLaserCorrection>& xmlCalibration);
  bool verifyCalibrationData(bool checkIncompleteCycles);

  vtkRollingDataValidator();
  ~vtkRollingDataValidator();

protected:
  std::vector<TypeValueDataPair> accumulatedDataComparable;
  std::vector<long> beginPositionsComparable;
  bool fullCycleComparableExists = false;
  std::vector<std::map<std::string, double>> xmlFileCorrections;
  GpsTopOfHourValues gpsTopOfHourValues;

private:
  void findBeginPositionsComparable();
  std::tuple<int, std::vector<int>> getCycleNumberAndIdxs(const int laser_id, const std::string &field);
  double getFieldValueScale(const std::string &field);
  double getThresholdForField(const std::string &field);
  double resolveStreamValue(const std::vector<TypeValueDataPair> &streamDataCycle, const int laser_id, const std::string &field);

  const std::vector<char> excludeMarkers = {'H','M','S','D','N','Y','G','T','V'};
  static const long expectedLengthComparable = 1820;
  const std::vector<std::string> fields = {
    "vert_correction", 
    "rot_correction", 
    "dist_correction", 
    "dist_correction_x", 
    "dist_correction_y", 
    "vert_offset_correction", 
    "horiz_offset_correction", 
    "focal_distance", 
    "focal_slope", 
    "min_intensity", 
    "max_intensity"
  };

};
#endif // VTKROLLINGDATAVALIDATOR_H
