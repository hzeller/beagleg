/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * (c) 2013, 2016 Henner Zeller <h.zeller@acm.org>
 *
 * This file is part of BeagleG. http://github.com/hzeller/beagleg
 *
 * BeagleG is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * BeagleG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with BeagleG.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "gcode-parser.h"

#include <unistd.h>
#include <sys/types.h>
#include <stdio.h>

#include "common/logging.h"

bool GCodeParser::Config::LoadParams(const std::string &filename) {
  if (parameters == NULL) {
    Log_error("No parameters to load into.");
    return false;
  }

  (*parameters)["5220"] = 1.0f;  // Only non-zero default.

  FILE *fp = fopen(filename.c_str(), "r");
  if (!fp) {
    Log_error("Unable to read param file %s (%s)",
              filename.c_str(), strerror(errno));
    return false;
  }

  int pcount = 0;
  char line[256];
  while (fgets(line, sizeof(line), fp)) {
    char name[256];
    float value;
    if (sscanf(line, "%s %f", name, &value) == 2) {
      // Technically, we should ignore parameters which are not coming in
      // order according to RS274NGC. But that sounds like a non-userfriendly
      // restriction.
      (*parameters)[name] = value;
      ++pcount;
    }
  }
  Log_debug("Loaded %d parameters from %s", pcount, filename.c_str());
  fclose(fp);
  return true;
}

bool GCodeParser::Config::SaveParams(const std::string &filename) {
  if (parameters == NULL) {
    Log_error("No parameters to save.");
    return false;
  }
  const std::string tmp_name = filename + ".tmp";
  // create new param file
  FILE *fp = fopen(tmp_name.c_str(), "w");
  if (!fp) {
    const int err = errno;
    Log_error("Unable to write param file %s (%s)", tmp_name.c_str(),
              strerror(err));
    if (err == EACCES) {
      Log_error("Permission problem: maybe because we run as uid=%d, gid=%d ?",
                getuid(), getgid());
    }
    return false;
  }

  int pcount = 0;
  // We have to go through the list twice. The numeric parameters need to be
  // stored in numerical order, followed by all the alphanumeric fields.
  //
  // Given that we are using a string-key in the map, the numeric fields are
  // sorted alphanumerically, which is not the same as numerically. So we
  // simply copy them to a temporary structure that sorts them numerically.
  std::map<int, float> numeric_params;
  for (const auto name_value : *parameters) {
    if (!isdigit(name_value.first[0]))
      break;
    numeric_params[atoi(name_value.first.c_str())] = name_value.second;
  }

  for (const auto num_value : numeric_params) {
    if (num_value.first == 0)
      continue;  // Never write this parameter. It should always be zero
    if (num_value.second != 0) {
      fprintf(fp, "%i\t%f\n", num_value.first, num_value.second);
      ++pcount;
    }
  }

  // Now, all the non-numeric parmeters
  int start_alpha = pcount;
  for (const auto name_value : *parameters) {
    if (isdigit(name_value.first[0])) continue;  // Numeric: already written
    if (name_value.first[0] != '_') continue;    // Only write global parameters
    if (name_value.second == 0) continue;        // Don't write boring zeroes.
    if (pcount == start_alpha) {
      fprintf(fp, "\n# Alphanumeric global parameters\n");
    }
    fprintf(fp, "%s\t%f\n", name_value.first.c_str(), name_value.second);
    ++pcount;
  }
  Log_debug("Saving %d parameters to %s", pcount, filename.c_str());

  if (fflush(fp) == 0 && fdatasync(fileno(fp)) == 0 && fclose(fp) == 0) {
    rename(filename.c_str(), (filename + ".bak").c_str());
    if (rename(tmp_name.c_str(), filename.c_str()) == 0)
      return true;
  }
  Log_error("Trouble writing parameter file %s (%s)", filename.c_str(),
            strerror(errno));
  return false;
}
