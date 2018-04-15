/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * (c) 2016 Henner Zeller <h.zeller@acm.org>
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

// Visualize the machine path. NOT pretty, just a quick hack.
// Right now merely a development aid, but might in the future be a
// toplevel program we can provide as tool

/*
  TODO
  3d projection
  Bounding box: should be switchable: range of machine or size of object.
*/

#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>

#include "common/logging.h"
#include "gcode-parser/gcode-parser.h"

#include "config-parser.h"
#include "gcode-machine-control.h"
#include "motor-operations.h"
#include "spindle-control.h"

namespace {
extern const char *viridis_colors[];
extern const char *measureLinePS;
}

// Simple gcode visualizer. Takes the gcode and shows its range.
// Takes two passes: first determines the bounding box to show axes, second
// draws within these.
class GCodePrintVisualizer : public GCodeParser::EventReceiver {
public:
  GCodePrintVisualizer(FILE *file, bool show_ijk, float scale)
    : file_(file), show_ijk_(show_ijk), scale_(scale),
      segment_count_(0), pass_(1),
      prefer_inch_display_(false) {
  }

  // We have multiple passes to determine ranges first.
  // Pass 1 - preparation, pass 2 - writing.
  void SetPass(int p) { pass_ = p; }

  void set_speed_factor(float f) final {}
  void set_temperature(float f) final {}
  void set_fanspeed(float speed) final {}
  void wait_temperature() final {}
  void motors_enable(bool b) final {}
  void go_home(AxisBitmap_t axes) final {
    // TODO: this might actually be a different corner of machine.
    if (pass_ == 2) fprintf(file_, "stroke 0 0 moveto  %% G28\n");
  }
  void inform_origin_offset(const AxesRegister& axes) final {}
  void dwell(float value) final { }
  bool rapid_move(float feed, const AxesRegister &axes) final {
    return coordinated_move(feed, axes);
  }
  bool coordinated_move(float feed, const AxesRegister &axes) final {
    if (pass_ == 1) {
      RememberMinMax(axes);
    } else {
      ++segment_count_;
      fprintf(file_, "%f %f lineto\n", axes[AXIS_X], axes[AXIS_Y]);
      if (segment_count_ % 256 == 0) {
        // Flush graphic context.
        fprintf(file_, "currentpoint\nstroke\nmoveto\n");
      }
    }
    return true;
  }

  void arc_move(float feed_mm_p_sec,
                GCodeParserAxis normal_axis, bool clockwise,
                const AxesRegister &start,
                const AxesRegister &center,
                const AxesRegister &end) final {
    if (pass_ == 2 && show_ijk_) {
      fprintf(file_, "currentpoint currentpoint stroke\n"
              "gsave\n\tmoveto [0.5] 0 setdash 0.1 setlinewidth 0 0 0.9 setrgbcolor\n"
              "\t%f %f lineto %f %f lineto stroke\n"
              "\t[] 0 setdash\ngrestore moveto %% show radius\n",
              center[AXIS_X], center[AXIS_Y], end[AXIS_X], end[AXIS_Y]);
    }
    EventReceiver::arc_move(feed_mm_p_sec, normal_axis, clockwise,
                            start, center, end);
  }

  void spline_move(float feed_mm_p_sec,
                   const AxesRegister &start,
                   const AxesRegister &cp1, const AxesRegister &cp2,
                   const AxesRegister &end) final {
    if (pass_ == 2 && show_ijk_) {
      fprintf(file_, "currentpoint stroke\n"
              "gsave\n\t[0.5] 0 setdash 0.1 setlinewidth 0 0 0.9 setrgbcolor\n"
              "\t%f %f moveto %f %f lineto stroke\n"
              "\t%f %f moveto %f %f lineto stroke\n"
              "\t[] 0 setdash\ngrestore moveto %% control points\n",
              start[AXIS_X], start[AXIS_Y], cp1[AXIS_X], cp2[AXIS_Y],
              cp2[AXIS_X], cp2[AXIS_Y], end[AXIS_X], end[AXIS_Y]);
    }
    EventReceiver::spline_move(feed_mm_p_sec, start, cp1, cp2, end);
  }

  void gcode_command_done(char letter, float val) final {
    // Remember if things were set to inch or metric, so that we can
    // show dimensions in preferred units.
    if (letter == 'G') {
      if (val == 21) {
        prefer_inch_display_ = false;
      } else if (val == 20) {
        prefer_inch_display_ = true;
      }
    }
  }

  const char *unprocessed(char letter, float value, const char *remain) final {
    return NULL;
  }

  void gcode_start(GCodeParser *parser) final {
    if (pass_ == 2) {
      fprintf(file_, "\n%% -- Path generated from GCode.\n");
      fprintf(file_, "0.1 setlinewidth 0 0 0 setrgbcolor\n0 0 moveto\n");
    }
  }

  void gcode_finished(bool end_of_stream) final {
    if (pass_ == 2 && end_of_stream) {
      fprintf(file_, "stroke\n");
    }
  }

  void GetDimensions(float *x, float *y, float *width, float *height) {
    *x = min_[AXIS_X];
    *y = min_[AXIS_Y];
    *width = max_[AXIS_X]-min_[AXIS_X];
    *height = max_[AXIS_Y]-min_[AXIS_Y];
  }

  float GetDiagonalLength() {
    return hypotf(max_[AXIS_X] - min_[AXIS_X], max_[AXIS_Y] - min_[AXIS_Y]);
  }

  void PrintShowRange(float margin) {
    fprintf(file_, "\n%% -- Print Dimensions\n");
    fprintf(file_, "%s", measureLinePS);
    fprintf(file_, "0 0 0.8 setrgbcolor 0.2 setlinewidth\n");
    // Font-size dependent on overall drawing size.
    fprintf(file_, "/Helvetica findfont %.1f scalefont setfont\n",
            std::max(1.0f, 0.015f * GetDiagonalLength()));
    fprintf(file_, "\n%% -- Dotted box around item\n");
    fprintf(file_, "%f %f moveto "
            "%f %f lineto stroke %% width\n",
            min_[AXIS_X], max_[AXIS_Y] + margin,
            max_[AXIS_X], max_[AXIS_Y] + margin);
    fprintf(file_, "gsave 0.5 setgray 0.1 setlinewidth [1] 0 setdash\n");
    fprintf(file_, "%f %f moveto %f %f lineto stroke\n",
            min_[AXIS_X], min_[AXIS_Y],
            min_[AXIS_X], max_[AXIS_Y]);
    fprintf(file_, "%f %f moveto %f %f lineto stroke\n",
            max_[AXIS_X], min_[AXIS_Y],
            max_[AXIS_X], max_[AXIS_Y]);
    fprintf(file_, "%f %f moveto %f %f lineto stroke\n",
            min_[AXIS_X], min_[AXIS_Y],
            max_[AXIS_X], min_[AXIS_Y]);
    fprintf(file_, "%f %f moveto %f %f lineto stroke\n",
            min_[AXIS_X], max_[AXIS_Y],
            max_[AXIS_X], max_[AXIS_Y]);
    fprintf(file_, "stroke grestore\n");
    const float df = prefer_inch_display_ ? 25.4f : 1.0f;  // display factor
    const int resolution = prefer_inch_display_ ? 3 : 2;
    const char *measure_unit = prefer_inch_display_ ? "\"" : "mm";
    fprintf(file_, "%f %f moveto (%.*f) (%.*f%s) (%.*f) %f MeasureLine\n",
            min_[AXIS_X], max_[AXIS_Y] + margin,
            resolution, min_[AXIS_X]/df,
            resolution, (max_[AXIS_X]-min_[AXIS_X])/df, measure_unit,
            resolution, max_[AXIS_X]/df,
            (max_[AXIS_X]-min_[AXIS_X]));

    fprintf(file_, "gsave %f %f translate -90 rotate 0 0 moveto (%.*f) (%.*f%s) (%.*f) %f MeasureLine grestore\n",
            max_[AXIS_X] + margin, max_[AXIS_Y],
            resolution, max_[AXIS_Y]/df,
            resolution, (max_[AXIS_Y]-min_[AXIS_Y])/df, measure_unit,
            resolution, min_[AXIS_Y]/df,
            (max_[AXIS_Y]-min_[AXIS_Y]));
  }

  void ShowHomePos() {
    fprintf(file_, "0 1 1 setrgbcolor 0.4 setlinewidth\n");
    fprintf(file_, "0 0 moveto 0 0 0.5 0 360 arc closepath "
            "gsave 0 setgray fill grestore stroke %% This is the home pos.\n");
  }

  void PrintPostscriptBoundingBox(float margin_x, float margin_y) {
    fprintf(file_, "%%!PS-Adobe-3.0 EPSF-3.0\n"
            "%%%%BoundingBox: %d %d %d %d\n",
            ToPoint(scale_ * (min_[AXIS_X] - 10)),
            ToPoint(scale_ * (min_[AXIS_Y] - 10)),
            ToPoint(scale_ * (max_[AXIS_X] + 10 + margin_x)),
            ToPoint(scale_ * (max_[AXIS_Y] + 10 + margin_y)));
  }

private:
  void RememberMinMax(const AxesRegister &axes) {
    if (axes[AXIS_X] < min_[AXIS_X]) min_[AXIS_X] = axes[AXIS_X];
    if (axes[AXIS_Y] < min_[AXIS_Y]) min_[AXIS_Y] = axes[AXIS_Y];
    if (axes[AXIS_Z] < min_[AXIS_Z]) min_[AXIS_Z] = axes[AXIS_Z];
    if (axes[AXIS_X] > max_[AXIS_X]) max_[AXIS_X] = axes[AXIS_X];
    if (axes[AXIS_Y] > max_[AXIS_Y]) max_[AXIS_Y] = axes[AXIS_Y];
    if (axes[AXIS_Z] > max_[AXIS_Z]) max_[AXIS_Z] = axes[AXIS_Z];
  }

  static int ToPoint(float mm) {
    return roundf(mm / 25.4 * 72);
  }

  FILE *const file_;
  const bool show_ijk_;
  const float scale_;

  AxesRegister min_;
  AxesRegister max_;
  unsigned int segment_count_;
  int pass_;
  bool prefer_inch_display_;
};

// Taking the low-level motor operations and visualize them. Uses color
// to visualize speed. Needs two passes - first pass determines available
// speed range, second outputs PostScript with colored segments.
// This also helps do determine if things line up properly with what the gcode
// parser spits out.
class MotorOperationsPrinter : public MotorOperations {
public:
  MotorOperationsPrinter(FILE *file, const MachineControlConfig &config,
                         float tool_dia, bool show_speeds)
    : file_(file), config_(config), show_speeds_(show_speeds), segment_count_(0),
      min_v_(1e6), max_v_(-1e6), pass_(0), color_segment_length_(1),
      last_color_index_(-1) {
    fprintf(file_, "\n%% -- Machine path. %s\n",
            show_speeds ? "Visualizing travel speeds" : "Simple.");
    fprintf(file_, "%% Note, larger tool diameters slow down "
            "PostScript interpreters a lot!\n");
    fprintf(file_, "/tool-diameter %.2f def\n\n", tool_dia);

    fprintf(file_, "tool-diameter setlinewidth\n");
    fprintf(file_, "0 0 moveto\n");
    if (!show_speeds_) {
      fprintf(file_, "0.6 0.6 0.6 setrgbcolor\n");
    }
  }

  ~MotorOperationsPrinter() {
    fprintf(file_, "stroke %% Finished Machine Pathstroke\n");
  }

  void SetPass(int p) {
    pass_ = p;
    if (pass_ == 2) {
      min_color_range_ = min_v_ + 0.1 * (max_v_ - min_v_);
      max_color_range_ = max_v_ - 0.1 * (max_v_ - min_v_);
      fprintf(stderr, "Speed: [%.2f..%.2f]; Coloring span [%.2f..%.2f]\n",
              min_v_, max_v_, min_color_range_, max_color_range_);
    }
  }

  void RememberMinMax(float v) {
    if (v > max_v_) max_v_ = v;
    if (v < min_v_) min_v_ = v;
  }

  void Enqueue(const LinearSegmentSteps &param) final {
    int dominant_axis = 0;
    for (int i = 1; i < BEAGLEG_NUM_MOTORS; ++i) {
      if (abs(param.steps[i]) > abs(param.steps[dominant_axis]))
        dominant_axis = i;
    }
    if (config_.steps_per_mm[dominant_axis] == 0 ||
        param.steps[dominant_axis] == 0) {
      return;  // Nothing really to do.
    }

    switch (pass_) {
    case 1: {
      // A very short diagnoal move, quantized to steps has a speed of sqrt(2);
      // let's not include these in the min/max calculation.
      if (abs(param.steps[AXIS_X])
          + abs(param.steps[AXIS_Y])
          + abs(param.steps[AXIS_Z]) <= 3)
        break;  // don't include super-short segments in the MinMax

      const float dx_mm = param.steps[AXIS_X] / config_.steps_per_mm[AXIS_X];
      const float dy_mm = param.steps[AXIS_Y] / config_.steps_per_mm[AXIS_Y];
      const float dz_mm = param.steps[AXIS_Z] / config_.steps_per_mm[AXIS_Z];
      const float segment_len = sqrtf(dx_mm*dx_mm + dy_mm*dy_mm + dz_mm*dz_mm);
      // The step speed is given by the dominant axis; however the actual
      // segment speed depends on the actual travel in euclidian space.
      const float segment_speed_factor = segment_len /
        (abs(param.steps[dominant_axis]) / config_.steps_per_mm[dominant_axis]);

      RememberMinMax(segment_speed_factor *
                     (param.v0 / config_.steps_per_mm[dominant_axis]));
      RememberMinMax(segment_speed_factor *
                     (param.v1 / config_.steps_per_mm[dominant_axis]));
    }
      break;
    case 2:
      PrintSegment(param, dominant_axis);
      break;
    }
  }

  // Set the length of how long the smallest colored range should be.
  void SetColorSegmentLength(float length) {
    color_segment_length_ = length;
  }

  void PrintSegment(const LinearSegmentSteps &param, int dominant_axis) {
    const float dx_mm = param.steps[AXIS_X] / config_.steps_per_mm[AXIS_X];
    const float dy_mm = param.steps[AXIS_Y] / config_.steps_per_mm[AXIS_Y];
    const float dz_mm = param.steps[AXIS_Z] / config_.steps_per_mm[AXIS_Z];

    if (show_speeds_) {
      fprintf(file_, "%% dx: %f dy: %f %s (speed: %.1f->%.1f)\n", dx_mm, dy_mm,
              param.v0 == param.v1 ? "; steady move" :
              ((param.v0 < param.v1) ? "; accel" : "; decel"),
              param.v0/config_.steps_per_mm[dominant_axis],
              param.v1/config_.steps_per_mm[dominant_axis]);
      const float segment_len = sqrtf(dx_mm*dx_mm + dy_mm*dy_mm + dz_mm*dz_mm);
      if (segment_len == 0)
        return;  // If not in x/y plane.
      // The step speed is given by the dominant axis; however the actual
      // physical speed depends on the actual travel in euclidian space.
      const float segment_speed_factor = segment_len /
        (abs(param.steps[dominant_axis]) / config_.steps_per_mm[dominant_axis]);
      int segments = 1;
      if (param.v0 != param.v1) {
        segments = segment_len / color_segment_length_;
        if (segments == 0) segments = 1;
      }
      for (int i = 0; i < segments; ++i) {
        float v = param.v0 + i * (param.v1 - param.v0)/segments;
        v /= config_.steps_per_mm[dominant_axis];
        v *= segment_speed_factor;
        int col_idx;
        if (v < min_color_range_) col_idx = 0;
        else if (v > max_color_range_) col_idx = 255;
        else {
          col_idx = roundf(255.0 * (v - min_color_range_)
                           / (max_color_range_-min_color_range_));
        }
        assert(col_idx >= 0 && col_idx < 256);
        if (col_idx != last_color_index_) {
          fprintf(file_, "%s setrgbcolor ", viridis_colors[col_idx]);
          last_color_index_ = col_idx;
        }
        fprintf(file_, "%f %f rlineto ",
                dx_mm/segments, dy_mm/segments);
        fprintf(file_, "currentpoint stroke moveto");
        fprintf(file_, " %% %.1f mm/s [%s]", v,
                param.v0 == param.v1 ? "=" : (param.v0 < param.v1 ? "^" : "v"));
        if (i == 0)
          fprintf(file_, " (%d,%d,%d)",
                  param.steps[AXIS_X], param.steps[AXIS_Y], param.steps[AXIS_Z]);
        fprintf(file_, "\n");
      }
    } else {
      fprintf(file_, "%f %f rlineto\n", dx_mm, dy_mm);
      ++segment_count_;
      if (segment_count_ % 256 == 0) {
        fprintf(file_, "currentpoint\nstroke\nmoveto\n");
      }
    }
  }

  void MotorEnable(bool on) final {}
  void WaitQueueEmpty() final {}
  bool GetPhysicalStatus(PhysicalStatus *status) final { return false; }

  void PrintColorLegend(float x, float y, float width) {
    if (min_color_range_ >= max_color_range_)
      return;
    const float barheight = std::min(8.0, 0.05 * width);
    const float fontsize = std::max(1.0, barheight * 0.75);
    fprintf(file_, "\n%% Color range visualization\n");
    fprintf(file_, "gsave /Helvetica findfont %.2f scalefont setfont\n",
            fontsize);
    fprintf(file_, "0 setgray 0.1 setlinewidth\n");
    fprintf(file_, "%.1f %.1f moveto (mm/s) show\n", x+width + 1, y-fontsize/2);
    fprintf(file_, "%.1f %.1f moveto 0 %.1f rlineto 0 0.5 rmoveto"
            "(<=%.1f) dup stringwidth pop 2 div neg 0 rmoveto show stroke\n",
            x, y, barheight, min_color_range_);

    const int kLegendPartitions = 6;
    for (int i = 1; i < kLegendPartitions; ++i) {
      float val = i * (max_color_range_-min_color_range_)/kLegendPartitions;
      float pos = i * width/kLegendPartitions;
      fprintf(file_, "%.1f %.1f moveto 0 %.1f rlineto 0 0.5 rmoveto"
              "(%.1f) dup stringwidth pop 2 div neg 0 rmoveto show stroke\n",
              x + pos, y, barheight, min_color_range_ + val);
    }
    fprintf(file_, "%.1f %.1f moveto 0 %.1f rlineto 0 0.5 rmoveto"
            "(>=%.1f) dup stringwidth pop 2 div neg 0 rmoveto show stroke\n",
            x+width, y, barheight, max_color_range_);

    fprintf(file_, "\n%% -- paint gradient\n");
    const float step = width/256;
    fprintf(file_, "%.1f setlinewidth %.1f %.1f moveto\n", barheight, x, y);
    for (int i = 0; i < 256; ++i) {
      // little overlap of 0.1, seems that the postscript interpreter otherwise
      // might leave little gaps.
      fprintf(file_, "%s setrgbcolor %f 0.1 add 0 rlineto currentpoint stroke moveto -0.1 0 rmoveto\n",
              viridis_colors[i], step);
    }
    fprintf(file_, "0 0 0 setrgbcolor  %% Done with gradient\n");

    // Show min/max range.
    fprintf(file_, "/Helvetica findfont %.1f scalefont setfont\n",
            fontsize * 0.75);
    fprintf(file_, "%.1f %.1f moveto (max %.1f mm/s) dup "
            "stringwidth pop neg 0 rmoveto show\n",
            x+width + 1, y-barheight, max_v_);
    fprintf(file_, "%.1f %.1f moveto (min %.1f mm/s) show\n",
            x, y-barheight, min_v_);

    fprintf(file_, "grestore\n");
  }

private:
  FILE *const file_;
  const MachineControlConfig &config_;
  const bool show_speeds_;
  unsigned segment_count_;
  float min_v_, max_v_;
  float min_color_range_, max_color_range_;
  int pass_;
  float color_segment_length_;
  int last_color_index_;

  MotorOperationsPrinter(const MotorOperationsPrinter &);
};

static int usage(const char *progname) {
  fprintf(stderr, "Usage: %s [options] <gcode-file>\n"
          "Options:\n"
          "\t-o <output-file>  : Name of output file; stdout default.\n"
          "\t-c <config>       : Machine config\n"
          "\t-T <tool-diameter>: Tool diameter in mm.\n"
          "\t-t <threshold-angle> : Threshold angle for accleration opt\n"
          "\t-s                : Visualize speeds\n"
          "\t-D                : show dimensions\n"
          "\t-S<factor>        : Scale the output (e.g. to fit on page)\n"
          "\t-i                : Toggle show IJK control lines\n"
          "Without config, only GCode path is shown; with config also the\n"
          "actual machine path.\n", progname);
  return 1;
}

static bool ParseFile(GCodeParser *parser, const char *filename, bool do_reset) {
  const int fd = open(filename, O_RDONLY);
  if (fd < 0) {
    fprintf(stderr, "Cannot open %s\n", filename);
    return false;
  }

  parser->ParseStream(fd, stderr);
  // Make sure to reset parser properly.
  if (do_reset) parser->ParseLine("G28 M02", stderr);  // M02 needs to be last.

  close(fd);
  return true;
}

int main(int argc, char *argv[]) {
  const char *config_file = NULL;
  FILE *output_file = stdout;
  float tool_diameter_mm = 2;
  bool show_dimensions = true;
  float threshold_angle = 0;
  bool show_speeds = false;
  bool range_check = false;
  bool show_ijk = true;
  float scale = 1.0f;

  int opt;
  while ((opt = getopt(argc, argv, "o:c:T:Dt:srS:i")) != -1) {
    switch (opt) {
    case 'o':
      output_file = fopen(optarg, "w");
      break;
    case 't':
      threshold_angle = (float)atof(optarg);
      break;
    case 'c':
      config_file = strdup(optarg);
      break;
    case 'D':
      show_dimensions = true;
      break;
    case 'T':
      tool_diameter_mm = atof(optarg);
      break;
    case 's':
      show_speeds = true;
      break;
    case 'S':
      scale = atof(optarg);
      break;
    case 'i':
      show_ijk = !show_ijk;
      break;
    case 'r':
      range_check = true;
      break;
    default:
      return usage(argv[0]);
    }
  }

  if (optind >= argc)
    return usage(argv[0]);

  if (output_file == NULL) {
    fprintf(stderr, "Could not create output file.\n");
    return 1;
  }

  const float printMargin = 2 + tool_diameter_mm/2;

  Log_init("/dev/null");

  // TODO(hzeller): only parse the file once, so that we can read from stdin.
  const char *filename = argv[optind];

  GCodeParser::Config parser_cfg;
  GCodeParser::Config::ParamMap parameters;  // TODO: read from file ?
  parser_cfg.parameters = &parameters;

  GCodePrintVisualizer gcode_printer(output_file, show_ijk, scale);
  gcode_printer.SetPass(1);
  GCodeParser gcode_viz_parser(parser_cfg, &gcode_printer, false);
  ParseFile(&gcode_viz_parser, filename, true);

  gcode_printer.PrintPostscriptBoundingBox(printMargin,
                                           printMargin + (show_speeds ? 15 : 0));

  fprintf(output_file, "72 25.4 div dup scale  %% Numbers mean millimeter\n");
  if (scale != 1.0f) {
    fprintf(output_file, "%f %f scale  %% Scale for page fit\n", scale, scale);
  }
  if (show_dimensions) {
    gcode_printer.PrintShowRange(printMargin);
  }

  if (config_file) {
    struct MachineControlConfig machine_config;
    ConfigParser config_parser;
    if (!config_parser.SetContentFromFile(config_file)) {
      fprintf(stderr, "Cannot read config file '%s'\n", config_file);
      return 1;
    }
    if (!machine_config.ConfigureFromFile(&config_parser)) {
      fprintf(stderr, "Exiting. Parse error in configuration file '%s'\n",
              config_file);
      return 1;
    }
    machine_config.threshold_angle = threshold_angle;

    // This is not connected to any machine. Don't assume homing, but
    // at least extract the home position from the config.
    machine_config.require_homing = false;
    for (const GCodeParserAxis axis : AllAxes()) {
      HardwareMapping::AxisTrigger trigger = machine_config.homing_trigger[axis];
      parser_cfg.machine_origin[axis] =
        (trigger & HardwareMapping::TRIGGER_MAX)
        ? machine_config.move_range_mm[axis]
        : 0;
    }

    machine_config.acknowledge_lines = false;
    machine_config.range_check = range_check;

    // We never initialize the hardware mapping from the config file, so
    // we have a convenient mapping of gcode-axis == motor-number
    // Non-initialized hardware mapping behaves like initialized
    HardwareMapping hardware;
    Spindle spindle;

    MotorOperationsPrinter motor_printer(output_file,
                                         machine_config, tool_diameter_mm,
                                         show_speeds);
    GCodeMachineControl *machine_control
      = GCodeMachineControl::Create(machine_config, &motor_printer,
                                    &hardware, &spindle, stderr);
    if (!machine_control) {
      // Ups, let's do it again with logging enabled.
      Log_init("/dev/stderr");
      Log_error("Cannot initialize machine:");
      GCodeMachineControl::Create(machine_config, &motor_printer,
                                  &hardware, &spindle, stderr);
    } else {
      machine_control->SetMsgOut(NULL);
      motor_printer.SetPass(1);
      GCodeParser parser(parser_cfg, machine_control->ParseEventReceiver(),
                         false);
      ParseFile(&parser, filename, true);

      machine_control->SetMsgOut(stderr);
      // The moves are segmented into colored segments. Don't make segments
      // unnecessarily small but somehow relate it to the maximum size of the
      // result.
      motor_printer.SetColorSegmentLength(gcode_printer.GetDiagonalLength()/100);
      motor_printer.SetPass(2);
      ParseFile(&parser, filename, false);

      if (show_speeds) {
        float x, y, w, h;
        gcode_printer.GetDimensions(&x, &y, &w, &h);
        motor_printer.PrintColorLegend(x, y + h + 15, w);
      }
    }
    delete machine_control;
  }

  gcode_printer.ShowHomePos();  // On top of machine path to be visible.
  // We print the gcode on top of the colored machine visualization.
  gcode_printer.SetPass(2);
  ParseFile(&gcode_viz_parser, filename, false);

  fprintf(output_file, "\nshowpage\n");

  fclose(output_file);

  return gcode_viz_parser.error_count() == 0 ? 0 : 1;
}

namespace {
// Perceptually nice colormap. See:
// https://bids.github.io/colormap/
// https://github.com/BIDS/colormap/blob/master/colormaps.py
const char *viridis_colors[] = {
  "0.267004 0.004874 0.329415",
  "0.268510 0.009605 0.335427",
  "0.269944 0.014625 0.341379",
  "0.271305 0.019942 0.347269",
  "0.272594 0.025563 0.353093",
  "0.273809 0.031497 0.358853",
  "0.274952 0.037752 0.364543",
  "0.276022 0.044167 0.370164",
  "0.277018 0.050344 0.375715",
  "0.277941 0.056324 0.381191",
  "0.278791 0.062145 0.386592",
  "0.279566 0.067836 0.391917",
  "0.280267 0.073417 0.397163",
  "0.280894 0.078907 0.402329",
  "0.281446 0.084320 0.407414",
  "0.281924 0.089666 0.412415",
  "0.282327 0.094955 0.417331",
  "0.282656 0.100196 0.422160",
  "0.282910 0.105393 0.426902",
  "0.283091 0.110553 0.431554",
  "0.283197 0.115680 0.436115",
  "0.283229 0.120777 0.440584",
  "0.283187 0.125848 0.444960",
  "0.283072 0.130895 0.449241",
  "0.282884 0.135920 0.453427",
  "0.282623 0.140926 0.457517",
  "0.282290 0.145912 0.461510",
  "0.281887 0.150881 0.465405",
  "0.281412 0.155834 0.469201",
  "0.280868 0.160771 0.472899",
  "0.280255 0.165693 0.476498",
  "0.279574 0.170599 0.479997",
  "0.278826 0.175490 0.483397",
  "0.278012 0.180367 0.486697",
  "0.277134 0.185228 0.489898",
  "0.276194 0.190074 0.493001",
  "0.275191 0.194905 0.496005",
  "0.274128 0.199721 0.498911",
  "0.273006 0.204520 0.501721",
  "0.271828 0.209303 0.504434",
  "0.270595 0.214069 0.507052",
  "0.269308 0.218818 0.509577",
  "0.267968 0.223549 0.512008",
  "0.266580 0.228262 0.514349",
  "0.265145 0.232956 0.516599",
  "0.263663 0.237631 0.518762",
  "0.262138 0.242286 0.520837",
  "0.260571 0.246922 0.522828",
  "0.258965 0.251537 0.524736",
  "0.257322 0.256130 0.526563",
  "0.255645 0.260703 0.528312",
  "0.253935 0.265254 0.529983",
  "0.252194 0.269783 0.531579",
  "0.250425 0.274290 0.533103",
  "0.248629 0.278775 0.534556",
  "0.246811 0.283237 0.535941",
  "0.244972 0.287675 0.537260",
  "0.243113 0.292092 0.538516",
  "0.241237 0.296485 0.539709",
  "0.239346 0.300855 0.540844",
  "0.237441 0.305202 0.541921",
  "0.235526 0.309527 0.542944",
  "0.233603 0.313828 0.543914",
  "0.231674 0.318106 0.544834",
  "0.229739 0.322361 0.545706",
  "0.227802 0.326594 0.546532",
  "0.225863 0.330805 0.547314",
  "0.223925 0.334994 0.548053",
  "0.221989 0.339161 0.548752",
  "0.220057 0.343307 0.549413",
  "0.218130 0.347432 0.550038",
  "0.216210 0.351535 0.550627",
  "0.214298 0.355619 0.551184",
  "0.212395 0.359683 0.551710",
  "0.210503 0.363727 0.552206",
  "0.208623 0.367752 0.552675",
  "0.206756 0.371758 0.553117",
  "0.204903 0.375746 0.553533",
  "0.203063 0.379716 0.553925",
  "0.201239 0.383670 0.554294",
  "0.199430 0.387607 0.554642",
  "0.197636 0.391528 0.554969",
  "0.195860 0.395433 0.555276",
  "0.194100 0.399323 0.555565",
  "0.192357 0.403199 0.555836",
  "0.190631 0.407061 0.556089",
  "0.188923 0.410910 0.556326",
  "0.187231 0.414746 0.556547",
  "0.185556 0.418570 0.556753",
  "0.183898 0.422383 0.556944",
  "0.182256 0.426184 0.557120",
  "0.180629 0.429975 0.557282",
  "0.179019 0.433756 0.557430",
  "0.177423 0.437527 0.557565",
  "0.175841 0.441290 0.557685",
  "0.174274 0.445044 0.557792",
  "0.172719 0.448791 0.557885",
  "0.171176 0.452530 0.557965",
  "0.169646 0.456262 0.558030",
  "0.168126 0.459988 0.558082",
  "0.166617 0.463708 0.558119",
  "0.165117 0.467423 0.558141",
  "0.163625 0.471133 0.558148",
  "0.162142 0.474838 0.558140",
  "0.160665 0.478540 0.558115",
  "0.159194 0.482237 0.558073",
  "0.157729 0.485932 0.558013",
  "0.156270 0.489624 0.557936",
  "0.154815 0.493313 0.557840",
  "0.153364 0.497000 0.557724",
  "0.151918 0.500685 0.557587",
  "0.150476 0.504369 0.557430",
  "0.149039 0.508051 0.557250",
  "0.147607 0.511733 0.557049",
  "0.146180 0.515413 0.556823",
  "0.144759 0.519093 0.556572",
  "0.143343 0.522773 0.556295",
  "0.141935 0.526453 0.555991",
  "0.140536 0.530132 0.555659",
  "0.139147 0.533812 0.555298",
  "0.137770 0.537492 0.554906",
  "0.136408 0.541173 0.554483",
  "0.135066 0.544853 0.554029",
  "0.133743 0.548535 0.553541",
  "0.132444 0.552216 0.553018",
  "0.131172 0.555899 0.552459",
  "0.129933 0.559582 0.551864",
  "0.128729 0.563265 0.551229",
  "0.127568 0.566949 0.550556",
  "0.126453 0.570633 0.549841",
  "0.125394 0.574318 0.549086",
  "0.124395 0.578002 0.548287",
  "0.123463 0.581687 0.547445",
  "0.122606 0.585371 0.546557",
  "0.121831 0.589055 0.545623",
  "0.121148 0.592739 0.544641",
  "0.120565 0.596422 0.543611",
  "0.120092 0.600104 0.542530",
  "0.119738 0.603785 0.541400",
  "0.119512 0.607464 0.540218",
  "0.119423 0.611141 0.538982",
  "0.119483 0.614817 0.537692",
  "0.119699 0.618490 0.536347",
  "0.120081 0.622161 0.534946",
  "0.120638 0.625828 0.533488",
  "0.121380 0.629492 0.531973",
  "0.122312 0.633153 0.530398",
  "0.123444 0.636809 0.528763",
  "0.124780 0.640461 0.527068",
  "0.126326 0.644107 0.525311",
  "0.128087 0.647749 0.523491",
  "0.130067 0.651384 0.521608",
  "0.132268 0.655014 0.519661",
  "0.134692 0.658636 0.517649",
  "0.137339 0.662252 0.515571",
  "0.140210 0.665859 0.513427",
  "0.143303 0.669459 0.511215",
  "0.146616 0.673050 0.508936",
  "0.150148 0.676631 0.506589",
  "0.153894 0.680203 0.504172",
  "0.157851 0.683765 0.501686",
  "0.162016 0.687316 0.499129",
  "0.166383 0.690856 0.496502",
  "0.170948 0.694384 0.493803",
  "0.175707 0.697900 0.491033",
  "0.180653 0.701402 0.488189",
  "0.185783 0.704891 0.485273",
  "0.191090 0.708366 0.482284",
  "0.196571 0.711827 0.479221",
  "0.202219 0.715272 0.476084",
  "0.208030 0.718701 0.472873",
  "0.214000 0.722114 0.469588",
  "0.220124 0.725509 0.466226",
  "0.226397 0.728888 0.462789",
  "0.232815 0.732247 0.459277",
  "0.239374 0.735588 0.455688",
  "0.246070 0.738910 0.452024",
  "0.252899 0.742211 0.448284",
  "0.259857 0.745492 0.444467",
  "0.266941 0.748751 0.440573",
  "0.274149 0.751988 0.436601",
  "0.281477 0.755203 0.432552",
  "0.288921 0.758394 0.428426",
  "0.296479 0.761561 0.424223",
  "0.304148 0.764704 0.419943",
  "0.311925 0.767822 0.415586",
  "0.319809 0.770914 0.411152",
  "0.327796 0.773980 0.406640",
  "0.335885 0.777018 0.402049",
  "0.344074 0.780029 0.397381",
  "0.352360 0.783011 0.392636",
  "0.360741 0.785964 0.387814",
  "0.369214 0.788888 0.382914",
  "0.377779 0.791781 0.377939",
  "0.386433 0.794644 0.372886",
  "0.395174 0.797475 0.367757",
  "0.404001 0.800275 0.362552",
  "0.412913 0.803041 0.357269",
  "0.421908 0.805774 0.351910",
  "0.430983 0.808473 0.346476",
  "0.440137 0.811138 0.340967",
  "0.449368 0.813768 0.335384",
  "0.458674 0.816363 0.329727",
  "0.468053 0.818921 0.323998",
  "0.477504 0.821444 0.318195",
  "0.487026 0.823929 0.312321",
  "0.496615 0.826376 0.306377",
  "0.506271 0.828786 0.300362",
  "0.515992 0.831158 0.294279",
  "0.525776 0.833491 0.288127",
  "0.535621 0.835785 0.281908",
  "0.545524 0.838039 0.275626",
  "0.555484 0.840254 0.269281",
  "0.565498 0.842430 0.262877",
  "0.575563 0.844566 0.256415",
  "0.585678 0.846661 0.249897",
  "0.595839 0.848717 0.243329",
  "0.606045 0.850733 0.236712",
  "0.616293 0.852709 0.230052",
  "0.626579 0.854645 0.223353",
  "0.636902 0.856542 0.216620",
  "0.647257 0.858400 0.209861",
  "0.657642 0.860219 0.203082",
  "0.668054 0.861999 0.196293",
  "0.678489 0.863742 0.189503",
  "0.688944 0.865448 0.182725",
  "0.699415 0.867117 0.175971",
  "0.709898 0.868751 0.169257",
  "0.720391 0.870350 0.162603",
  "0.730889 0.871916 0.156029",
  "0.741388 0.873449 0.149561",
  "0.751884 0.874951 0.143228",
  "0.762373 0.876424 0.137064",
  "0.772852 0.877868 0.131109",
  "0.783315 0.879285 0.125405",
  "0.793760 0.880678 0.120005",
  "0.804182 0.882046 0.114965",
  "0.814576 0.883393 0.110347",
  "0.824940 0.884720 0.106217",
  "0.835270 0.886029 0.102646",
  "0.845561 0.887322 0.099702",
  "0.855810 0.888601 0.097452",
  "0.866013 0.889868 0.095953",
  "0.876168 0.891125 0.095250",
  "0.886271 0.892374 0.095374",
  "0.896320 0.893616 0.096335",
  "0.906311 0.894855 0.098125",
  "0.916242 0.896091 0.100717",
  "0.926106 0.897330 0.104071",
  "0.935904 0.898570 0.108131",
  "0.945636 0.899815 0.112838",
  "0.955300 0.901065 0.118128",
  "0.964894 0.902323 0.123941",
  "0.974417 0.903590 0.130215",
  "0.983868 0.904867 0.136897",
  "0.993248 0.906157 0.143936"
};

const char *measureLinePS =
  "/MeasureLine {\n"
  "    /width exch def\n"
  "    /rtext exch def\n"
  "    /ctext exch def\n"
  "    /ltext exch def\n"
  "    currentpoint\n"
  "    0 -2 rmoveto\n"
  "    0 4 rlineto\n"
  "    0 1 rmoveto ltext show\n"
  "    moveto width 0 rlineto\n"
  "    currentpoint\n"
  "    0 -2 rmoveto\n"
  "    0 4 rlineto\n"
  "    0 1 rmoveto rtext dup stringwidth pop neg 0 rmoveto show\n"
  "    exch width 2 div sub exch 1 add moveto\n"
  "    ctext dup stringwidth pop 2 div neg 0 rmoveto show\n"
  "    stroke\n"
  "} def\n";
}
