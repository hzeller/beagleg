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

// HACK.

// Visualize the machine path. NOT pretty, just a total hack to have
// some development visualization to gauge planner output.
// Right now merely a development aid, but might in the future be a
// toplevel program we can provide as tool, but we probably also want
// to output to SVG and ThreeJS

/*
  TODO
  /half/ Option: Not display movements to/from home
  Crash if not all axis defined (../minimal)
  Choose absolute size.

  Fill the view ? Currently we're taking all axes into account to accomodate
  -V iso. If we have a specific view, we might just scale a bit differently.
  Auto tool diameter choice.
  Choose plane of measuring text and line depending on view to be visible.
  Auto view: if things ever are only in 2D or with only minimal deviation,
    only show that.
  rotation anim: not overall yaw, but first yaw, then rest of rot.
  Put all data in one or more arrays to be able to execute more, and maybe
    be a bit more efficient in doing so.
  Put look-up table for color gradient in program to keep rest more compact.
  Show warning: NOT HOMED
  Multiple G54 should print multiple origins
  example with multiple origins going on
  make panel TOP RIGHT FRONT ISO
  GEB (Goedel Escher Bach)
  teapot. http://www.holmes3d.net/graphics/teapot/teapotCGA.bpt
*/

#include <assert.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <complex>
#include <functional>

#include "common/logging.h"
#include "gcode-parser/gcode-parser.h"

#include "motion-queue.h"
#include "config-parser.h"
#include "gcode-machine-control.h"
#include "motor-operations.h"
#include "spindle-control.h"
#include "hershey.h"

// This is mostly only really useful for debugging stuff.
#define SHOW_SPEED_DETAILS 0

// Not all of these have command line flags yet.
struct VisualizationOptions {
  bool show_ijk = true;
  bool show_speeds = false;
  bool show_laser_burn = false;  // experimental. If on, S-value->gcode-bright
  std::string show_title = "BeagleG ~ gcode2ps";

  // Not handled with commad line flags (yet)
  bool include_machine_bounding_box = false;
  bool include_machine_origin = true;    // in case we want it even if now bbox

  bool output_js_vertices = false;
  bool show_out_of_range = false;        // Show out of range in red.
  float relative_font_size = 1.0 / 40;   // Fraction of machine cube diagonal.
};

// G-code rendering
static constexpr char kGcodeMoveColor[] = "0 0 0";
static constexpr char kGcodeRapidMoveColor[] = "0.7 0.7 0.7";
static constexpr char kGcodeOriginMarkColor[] = "0.5 0.5 0.8";
static constexpr char kGcodeOriginTextColor[] = "0 0 0";
static constexpr char kGcodeGridColor[] = "0.7 0.7 0.9";

// Machine movement rendering.
static constexpr char kOutOfRangeColor[] = "1 0.5 0.5";
static constexpr char kMachineMoveColor[] = "0.6 0.6 0.6";

static constexpr char kInchFormat[] = "%.3f";        // w/o unit
static constexpr char kInchUnitFormat[] = "%.3f\"";  // w/ unit

static constexpr char kMillimeterFormat[] = "%.2f";         // w/o unit
static constexpr char kMillimeterUnitFormat[] = "%.2fmm";   // w/ unit

static const std::map<std::string, const char *> kNamedViews = {
  { "top",       "" }, // Nothing to do: default.
  { "front",     "-90 pitch-rotate3d % front view" },
  { "left",      "-90 pitch-rotate3d 90 yaw-rotate3d % left view"},
  { "right",     "-90 yaw-rotate3d -90 roll-rotate3d % right view"},
  { "isometric", ("-90 pitch-rotate3d  45 yaw-rotate3d  "
                  "35.264 pitch-rotate3d  % Isometric view")}
  // Dimetric ? Trimetric ?
};

namespace {
extern const char *viridis_colors[];
extern const char *kPSHeader;
}

// Our classes generally work in two phases. Use some symbolic names here for
// readability.
enum class ProcessingStep {
  Init,
  Preprocess,
  GenerateOutput
};

// Simple gcode visualizer. Takes the gcode and shows its range.
// Takes two passes: first determines the bounding box to show axes, second
// draws within these.
class GCodePrintVisualizer : public GCodeParser::EventReceiver {
public:
  GCodePrintVisualizer(FILE *file, const AxesRegister &machine_origin,
                       const AxesRegister &move_range,
                       const VisualizationOptions &options)
    : file_(file), machine_origin_(machine_origin),
      opts_(options) {
    if (!opts_.include_machine_bounding_box) {
      // Make sure that we capture the true ranges of things.
      // TODO: we can use this later to determine if such axis was used,
      // e.g. by comparing max < min
      min_[AXIS_X] = 1e6; max_[AXIS_X] = -1e6;
      min_[AXIS_Y] = 1e6; max_[AXIS_Y] = -1e6;
      min_[AXIS_Z] = 1e6; max_[AXIS_Z] = -1e6;
    }
    else {
      RememberMinMax(move_range);
    }
  }

  // We have multiple passes to determine ranges first.
  // Pass 1 - preparation, pass 2 - writing.
  void SetPass(ProcessingStep p) { pass_ = p; }

  void set_speed_factor(float f) final {}
  void set_temperature(float f) final {}
  void set_fanspeed(float speed) final {}
  void wait_temperature() final {}
  void motors_enable(bool b) final {}
  void go_home(AxisBitmap_t axes) final {
    if (pass_ == ProcessingStep::GenerateOutput) {
      fprintf(file_, "%f %f %f moveto3d  %% G28\n",
              machine_origin_[AXIS_X], machine_origin_[AXIS_Y],
              machine_origin_[AXIS_Z]);
    }
    if (opts_.include_machine_origin) RememberMinMax(machine_origin_);
    just_homed_ = true;
  }
  void inform_origin_offset(const AxesRegister& axes, const char *n) final {
    if (pass_ == ProcessingStep::GenerateOutput) {
      ShowNamedOrigin(axes, n);
#if 0
    // Should we print an origin marker here ?
    fprintf(stderr, "Got origin [%f,%f,%f] %s\n",
            axes[AXIS_X], axes[AXIS_Y], axes[AXIS_Z], n);
#endif
    }
  }
  void dwell(float value) final { }
  bool rapid_move(float feed, const AxesRegister &axes) final {
    return do_move(true, feed, axes);
  }

  bool coordinated_move(float feed, const AxesRegister &axes) final {
    return do_move(false, feed, axes);
  }

  bool do_move(bool rapid, float feed, const AxesRegister &axes) {
    // It is possible to just say G1 F100 without any coordinates. But that
    // causes a call to coordinated move. So if this happens right after a G28
    // we would still include that coordinate even though we don't mean to.
    // TODO: there needs to be a callback that just sets the feedrate.
    if (!opts_.include_machine_origin && axes == machine_origin_)
      return true;
    if (pass_ == ProcessingStep::Preprocess) {
      RememberMinMax(axes);
    } else {
      if (rapid != last_move_rapid_) {
        fprintf(file_, "%s switch-color %.3f setlinewidth %% %s move\n",
                rapid ? kGcodeRapidMoveColor : kGcodeMoveColor,
                rapid ? 0.0 : GetDiagonalLength() / 1000.0,
                rapid ? "G0 rapid" : "G1 coordinated");
        last_move_rapid_ = rapid;
      }
      const bool not_show_after_home = (just_homed_ &&
                                        !opts_.include_machine_origin);
      if (opts_.show_laser_burn && laser_max_ > laser_min_
          && laser_intensity_ != last_laser_intensity_) {
        const float scaled_intensity = (laser_intensity_ - laser_min_)
          / (laser_max_ - laser_min_);
        fprintf(file_, "%.2f setgray ", 1 - scaled_intensity);
        last_laser_intensity_ = laser_intensity_;
      }
      fprintf(file_, "%.3f %.3f %.3f %s\n",
              axes[AXIS_X], axes[AXIS_Y], axes[AXIS_Z],
              not_show_after_home ? "moveto3d % post-G28" : "lineto3ds");
      if (opts_.output_js_vertices)
        fprintf(stdout, " [%.3f, %.3f, %.3f, %s],\n",   // ThreeJS
                axes[AXIS_X], axes[AXIS_Y], axes[AXIS_Z],
                rapid ? "true":"false");
    }
    just_homed_ = false;
    return true;
  }

  bool arc_move(float feed_mm_p_sec,
                GCodeParserAxis normal_axis, bool clockwise,
                const AxesRegister &start,
                const AxesRegister &center,
                const AxesRegister &end) final {
    if (pass_ == ProcessingStep::GenerateOutput && opts_.show_ijk) {
      const float line_size = GetDiagonalLength() / 500.0;
      fprintf(file_,
              "stroke currentpoint3d\n"  // remember for after the place
              "currentpoint3d gsave moveto3d\n\t "   // save restore
              "[%.3f] 0 setdash %.3f setlinewidth 0.8 0.8 1 setrgbcolor\n"
              "\t%f %f %f lineto3d %f %f %f lineto3d stroke\n"
              "\tgrestore %% end show radius\n"
              "moveto3d %% go back to last currentpoint3d\n",
              2 * line_size, line_size,
              center[AXIS_X], center[AXIS_Y], center[AXIS_Z],
              end[AXIS_X], end[AXIS_Y], end[AXIS_Z]);
    }
    // Let the standard arc generation happen.
    return EventReceiver::arc_move(feed_mm_p_sec, normal_axis, clockwise,
                                   start, center, end);
  }

  bool spline_move(float feed_mm_p_sec,
                   const AxesRegister &start,
                   const AxesRegister &cp1, const AxesRegister &cp2,
                   const AxesRegister &end) final {
    if (pass_ == ProcessingStep::GenerateOutput && opts_.show_ijk) {
      const float line_size = GetDiagonalLength() / 500.0;
      fprintf(file_,
              "stroke currentpoint3d\n"  // remember for after the place
              "currentpoint3d gsave moveto3d\n\t "   // save restore
              "[%.3f] 0 setdash %.3f setlinewidth 0.8 0.8 1 setrgbcolor\n"
              "\t%f %f %f moveto3d %f %f %f lineto3d stroke\n"
              "\t%f %f %f moveto3d %f %f %f lineto3d stroke\n"
              "\tgrestore %% end show control points\n"
              "moveto3d %% going back to last currentpoint3d\n",
              2 * line_size, line_size,
              start[AXIS_X], start[AXIS_Y], start[AXIS_Z],
              // TODO(hzeller): are these points actually only 2D ?
              cp1[AXIS_X], cp1[AXIS_Y], start[AXIS_Z],
              cp2[AXIS_X], cp2[AXIS_Y], start[AXIS_Z],
              end[AXIS_X], end[AXIS_Y], end[AXIS_Z]);
    }
    return EventReceiver::spline_move(feed_mm_p_sec, start, cp1, cp2, end);
  }

  void gcode_command_done(char letter, float val) final {
    // Remember if things were set to inch or metric, so that we can
    // show dimensions in preferred units.
    if (letter == 'G') {
      if (val == 21 || val == 71) {
        prefer_inch_ = false;
      } else if (val == 20 || val == 70) {
        prefer_inch_ = true;
      }
    }
  }

  void change_spindle_speed(float value) final {
    // Hack to visualize brightness in lasing applications.
    laser_intensity_ = value;
    if (laser_intensity_ < laser_min_) laser_min_ = laser_intensity_;
    if (laser_intensity_ > laser_max_) laser_max_ = laser_intensity_;
  }

  const char *unprocessed(char letter, float value, const char *remain) final {
    return NULL;
  }

  void gcode_start(GCodeParser *parser) final {
    if (pass_ == ProcessingStep::GenerateOutput) {
      fprintf(file_, "\n%% -- Path generated from GCode.\n");
      fprintf(file_, "%.3f setlinewidth 0 0 0 setrgbcolor\n",
              GetDiagonalLength() / 1000.0);

      if (opts_.include_machine_origin) {
        fprintf(file_, "%f %f %f moveto3d  %% Machine origin but not homed.\n",
                machine_origin_[AXIS_X], machine_origin_[AXIS_Y],
                machine_origin_[AXIS_Z]);
      }

      if (opts_.output_js_vertices) {
        fprintf(stdout, "\n// x, y, z, is_rapid\nvar vertices = [\n");
      }
    }
  }

  void gcode_finished(bool end_of_stream) final {
    if (pass_ == ProcessingStep::Preprocess) {
      if (min_[AXIS_X] > max_[AXIS_X]) min_[AXIS_X] = max_[AXIS_X] = 0;
      if (min_[AXIS_Y] > max_[AXIS_Y]) min_[AXIS_Y] = max_[AXIS_Y] = 0;
      if (min_[AXIS_Z] > max_[AXIS_Z]) min_[AXIS_Z] = max_[AXIS_Z] = 0;
    }
    if (pass_ == ProcessingStep::GenerateOutput && end_of_stream) {
      fprintf(file_, "stroke\n");
      fprintf(file_, "%% -- Finished GCode Path.\n");

      if (opts_.output_js_vertices) fprintf(stdout, "];\n"); // ThreeJS
    }
  }

  void GetDimensions(float *x, float *y, float *width, float *height) {
    *width = *height = GetDiagonalLength();
    *x = -*width/2;
    *y = -*width/2;
  }

  static float euclid_distance(float x, float y, float z) {
    return sqrtf(x*x + y*y + z*z);
  }

  float GetDiagonalLength() {
    return euclid_distance(max_[AXIS_X] - min_[AXIS_X],
                           max_[AXIS_Y] - min_[AXIS_Y],
                           max_[AXIS_Z] - min_[AXIS_Z]);
  }

  void MeasureLine(float min, float max, float size,
                   bool show_metric,
                   std::function<void(bool do_line, float c1, float c2)> draw) {
    float width = max - min;
    if (width < 3 * size)
      return;
    draw(false, 0, -size); draw(true, 0, size);  // first tick.
    draw(false, 0, 0); draw(true, width, 0);     // full length
    draw(false, width, -size); draw(true, width, size);  // second tick.
    const char *fmt = show_metric ? kMillimeterUnitFormat : kInchUnitFormat;
    std::string measure = StringPrintf(fmt, show_metric ? width : width / 25.4);
    DrawText(measure, width/2, -size, TextAlign::kCenter, size, draw);
    // For the small start/end markers, don't include the unit for compatness.
    fmt = show_metric ? kMillimeterFormat : kInchFormat;
    measure = StringPrintf(fmt, show_metric ? min : min / 25.4);
    DrawText(measure, 0, size/10, TextAlign::kCenter, size * 0.6,
               [draw, size](bool d, float x, float y) {
                draw(d, -y, x);
              });
    measure = StringPrintf(fmt, show_metric ? max : max / 25.4);
    DrawText(measure, 0, 0, TextAlign::kCenter, size * 0.6,
              [draw, size, width](bool d, float x, float y) {
                draw(d, width-y+0.6*size, x);
              });
  }

  void PrintHeader(float margin, float eye_distance, int animation_frames) {
    fprintf(file_, "\n%% Center of display cube; origin of all rotations.\n");
    fprintf(file_, "/center-x %.3f def %% [%.1f %.1f]\n",
            (max_[AXIS_X] + min_[AXIS_X])/2, min_[AXIS_X], max_[AXIS_X]);
    fprintf(file_, "/center-y %.3f def %% [%.1f %.1f]\n",
            (max_[AXIS_Y] + min_[AXIS_Y])/2, min_[AXIS_Y], max_[AXIS_Y]);
    fprintf(file_, "/center-z %.3f def %% [%.1f %.1f]\n",
            (max_[AXIS_Z] + min_[AXIS_Z])/2, min_[AXIS_Z], max_[AXIS_Z]);
    // TODO(hzeller): this certainly needs to take scale and such into
    // account.
    fprintf(file_, "\n%% If larger than zero, shows perspective\n");
    fprintf(file_, "/perspective-eye-distance %.1f def\n",
            eye_distance > 0 ? eye_distance + GetDiagonalLength()/2 : -1);

    if (animation_frames > 0) {
      fprintf(file_, "%s", R"(
% Outputs these number of frames of a model rotationg around the vertical axis.
% Use e.g.
%    gs -q -dBATCH -dNOPAUSE -sDEVICE=png16m -sOutputFile=foo-%03d.png <ps-file>
% to convert to single images, then
%    convert foo-*.png animated.gif
% to create an animation. Note, huge files will make the postscript interpreter
% unhappy.
)");
      fprintf(file_, "/animation-frames %d def\n", animation_frames);
    }

    fprintf(file_, "%s", kPSHeader);
  }

  static float start_grid(float min_value, float grid) {
    const float offset = fmod(min_value, grid);
    const float start = min_value - offset;
    return start < min_value ? start + grid : start;
  }

  void DrawGrid(float d) {
    fprintf(file_, "\n%% -- Grid\n");
    fprintf(file_, "%s setrgbcolor 0 setlinewidth\n", kGcodeGridColor);
    for (float x = start_grid(min_[AXIS_X], d) ; x < max_[AXIS_X]; x += d) {
      fprintf(file_, "%.3f %.3f %.3f moveto3d %.3f %.3f %.3f lineto3d\n",
              x, min_[AXIS_Y], min_[AXIS_Z], x, max_[AXIS_Y], min_[AXIS_Z]);
    }
    for (float y = start_grid(min_[AXIS_Y], d) ; y < max_[AXIS_Y]; y += d) {
      fprintf(file_, "%.3f %.3f %.3f moveto3d %.3f %.3f %.3f lineto3d\n",
              min_[AXIS_X], y, min_[AXIS_Z], max_[AXIS_X], y, min_[AXIS_Z]);
    }
    fprintf(file_, "stroke %% end grid\n");
  }

  void PrintModelFrame() {
    const float size = GetDiagonalLength() / 30;

    // The dash corresponds to 1mm in real coordinates only if flat projected.
    // Maybe we want to adapt that for the projection ?
    fprintf(file_, "gsave %.2f setlinewidth\n", size/25);

    // -- this should be a loop
    fprintf(file_, "\n%% -- Dotted box around item\n");
    // bottom plane, remaining.
    fprintf(file_, "0.6 setgray [1] 0 setdash\n");
    fprintf(file_, "%f %f %f moveto3d %f %f %f lineto3d\n",
            max_[AXIS_X], min_[AXIS_Y], min_[AXIS_Z],
            max_[AXIS_X], max_[AXIS_Y], min_[AXIS_Z]);
    fprintf(file_, "%f %f %f moveto3d %f %f %f lineto3d\n",
            min_[AXIS_X], max_[AXIS_Y], min_[AXIS_Z],
            max_[AXIS_X], max_[AXIS_Y], min_[AXIS_Z]);

    // upper plane
    fprintf(file_, "%f %f %f moveto3d %f %f %f lineto3d\n",
            min_[AXIS_X], min_[AXIS_Y], max_[AXIS_Z],
            min_[AXIS_X], max_[AXIS_Y], max_[AXIS_Z]);
    fprintf(file_, "%f %f %f moveto3d %f %f %f lineto3d\n",
            max_[AXIS_X], min_[AXIS_Y], max_[AXIS_Z],
            max_[AXIS_X], max_[AXIS_Y], max_[AXIS_Z]);
    fprintf(file_, "%f %f %f moveto3d %f %f %f lineto3d\n",
            min_[AXIS_X], min_[AXIS_Y], max_[AXIS_Z],
            max_[AXIS_X], min_[AXIS_Y], max_[AXIS_Z]);
    fprintf(file_, "%f %f %f moveto3d %f %f %f lineto3d\n",
            min_[AXIS_X], max_[AXIS_Y], max_[AXIS_Z],
            max_[AXIS_X], max_[AXIS_Y], max_[AXIS_Z]);

    // corners
    fprintf(file_, "%f %f %f moveto3d %f %f %f lineto3d\n",
            min_[AXIS_X], max_[AXIS_Y], min_[AXIS_Z],
            min_[AXIS_X], max_[AXIS_Y], max_[AXIS_Z]);
    fprintf(file_, "%f %f %f moveto3d %f %f %f lineto3d\n",
            max_[AXIS_X], min_[AXIS_Y], min_[AXIS_Z],
            max_[AXIS_X], min_[AXIS_Y], max_[AXIS_Z]);
    fprintf(file_, "%f %f %f moveto3d %f %f %f lineto3d\n",
            max_[AXIS_X], max_[AXIS_Y], min_[AXIS_Z],
            max_[AXIS_X], max_[AXIS_Y], max_[AXIS_Z]);

    // Slightly thicker solid lines
    fprintf(file_, "stroke [] 0 setdash %.2f setlinewidth\n", size/20);

    // Primary axes.
    fprintf(file_, "\n%% -- Solid X/Y/Z marking min-axes...\n");
    fprintf(file_, "0.4 1 0.4 setrgbcolor "
            "%f %f %f moveto3d %f %f %f lineto3d stroke\n",
            min_[AXIS_X], min_[AXIS_Y], min_[AXIS_Z],
            min_[AXIS_X], max_[AXIS_Y], min_[AXIS_Z]);
    fprintf(file_, "1 0.4 0.4 setrgbcolor "
            "%f %f %f moveto3d %f %f %f lineto3d stroke\n",
            min_[AXIS_X], min_[AXIS_Y], min_[AXIS_Z],
            max_[AXIS_X], min_[AXIS_Y], min_[AXIS_Z]);
    fprintf(file_, "0.4 0.4 1 setrgbcolor "
            "%f %f %f moveto3d %f %f %f lineto3d stroke\n",
            min_[AXIS_X], min_[AXIS_Y], min_[AXIS_Z],
            min_[AXIS_X], min_[AXIS_Y], max_[AXIS_Z]);

    fprintf(file_, "grestore\n");

    if (opts_.output_js_vertices) {
      fprintf(stdout, "var item_cube = [");
      fprintf(stdout, "[%.3f, %.3f, %.3f], [%.3f, %.3f, %.3f]];\n",
              min_[AXIS_X], min_[AXIS_Y], min_[AXIS_Z],
              max_[AXIS_X], max_[AXIS_Y], max_[AXIS_Z]);
    }

  }

  void ShowMesaureLines() {
    fprintf(file_, "\n%% -- Measurement lines\n");
    const float size = opts_.relative_font_size * GetDiagonalLength();
    fprintf(file_, "%.1f setlinewidth\n", size/20);

    // Various functions to draw into directon of the axis on given
    // plane.
    // TODO(hzeller): depending on view angle, we might choose different
    // planes to draw stuff on.
    auto x_xy_draw = [this, size](bool do_line, float x, float y) {
      fprintf(file_, "%.3f %.3f %.3f %s\n",
              min_[AXIS_X] + x, min_[AXIS_Y] - 1.5*size + y,
              min_[AXIS_Z],
              do_line ? "lineto3d" : "moveto3d");
    };
    auto y_xy_draw = [this, size](bool do_line, float x, float y) {
      fprintf(file_, "%.3f %.3f %.3f %s\n",
              min_[AXIS_X] -y - 1.5*size,
              min_[AXIS_Y] + x, min_[AXIS_Z],
              do_line ? "lineto3d" : "moveto3d");
    };
    auto z_yz_draw = [this, size](bool do_line, float x, float y) {
      fprintf(file_, "%.3f %.3f %.3f %s\n",
              min_[AXIS_X], max_[AXIS_Y]+y+1.5*size,
              x + min_[AXIS_Z],
              do_line ? "lineto3d" : "moveto3d");
    };
#if 0
    auto z_xz_draw = [this, size](bool do_line, float x, float y) {
      fprintf(file_, "%.3f %.3f %.3f %s\n",
              min_[AXIS_X]-y-1.5*size, max_[AXIS_Y],
              x + min_[AXIS_Z],
              do_line ? "lineto3d" : "moveto3d");
    };
#endif
    if (!opts_.show_title.empty()) {
      fprintf(file_, "0.7 0.7 0.7 setrgbcolor\n");
      float text_size = size * 0.7;
      const float text_width = TextWidth(opts_.show_title, text_size);
      const float max_x_space = (max_[AXIS_X] - min_[AXIS_X]) - 2*size;
      if (text_width > max_x_space)
        text_size *= max_x_space / text_width;
      DrawText(opts_.show_title, size, 0.3*size,
               TextAlign::kLeft, text_size, x_xy_draw);
      fprintf(file_, "stroke\n");
    }
    fprintf(file_, "0.8 0 0 setrgbcolor               %% X-axis\n");
    MeasureLine(min_[AXIS_X], max_[AXIS_X], size, !prefer_inch_, x_xy_draw);
    fprintf(file_, "stroke 0 0.8 0 setrgbcolor        %% Y-Axis\n");
    MeasureLine(min_[AXIS_Y], max_[AXIS_Y], size, !prefer_inch_, y_xy_draw);
    fprintf(file_, "stroke 0 0 0.8 setrgbcolor        %% Z-Axis\n");
    MeasureLine(min_[AXIS_Z], max_[AXIS_Z], size, !prefer_inch_, z_yz_draw);
    fprintf(file_, "stroke 0 0 0 setrgbcolor\n");
  }

  void ShowNamedOrigin(const AxesRegister &origin, const char *named) {
    fprintf(file_, "\n%% -- Origin %s\n", named);
    const float size = opts_.relative_font_size * GetDiagonalLength() / 2;
    fprintf(file_, "currentpoint3d stroke gsave "
            "%s setrgbcolor 0 setlinewidth ",
            kGcodeOriginTextColor);
    DrawText(named, 0, size, TextAlign::kCenter, 1.5*size,
             [this, origin](bool d, float x, float y) {
               fprintf(file_, "%.3f %.3f %.3f %s\n",
                       origin[AXIS_X] + x, origin[AXIS_Y] + y, origin[AXIS_Z],
                       d ? "lineto3d" : "moveto3d");
             });
    fprintf(file_, "stroke %s setrgbcolor\n", kGcodeOriginMarkColor);
    const float x = origin[AXIS_X];
    const float y = origin[AXIS_Y];
    const float z = origin[AXIS_Z];

    // Octagon with bottom-left quadrant filled.
    fprintf(file_, "%.3f %.3f %.3f moveto3d\n", x, y, z);
    fprintf(file_, "%.3f %.3f %.3f lineto3d\n", x-size, y, z);
    fprintf(file_, "%.3f %.3f %.3f lineto3d\n", x-0.707*size, y-0.707*size, z);
    fprintf(file_, "%.3f %.3f %.3f lineto3d\n", x, y-size, z);
    fprintf(file_, "closepath fill\n");
    // remaining part, non-filled.
    fprintf(file_, "%.3f %.3f %.3f moveto3d\n", x, y - size, z);
    fprintf(file_, "%.3f %.3f %.3f lineto3d\n", x+0.707*size, y-0.707*size, z);
    fprintf(file_, "%.3f %.3f %.3f lineto3d\n", x+size, y, z);
    fprintf(file_, "%.3f %.3f %.3f lineto3d\n", x+0.707*size, y+0.707*size, z);
    fprintf(file_, "%.3f %.3f %.3f lineto3d\n", x, y+size, z);
    fprintf(file_, "%.3f %.3f %.3f lineto3d\n", x-0.707*size, y+0.707*size, z);
    fprintf(file_, "%.3f %.3f %.3f lineto3d\n", x-size, y, z);
    fprintf(file_, "stroke grestore moveto3d\n");
  }

  void ShowHomePos(const AxesRegister &origin) {
    fprintf(file_, "\n%% -- Machine home\n");
    const float size = GetDiagonalLength() / 200;  // 0.5%
    fprintf(file_, "currentpoint3d stroke gsave "
            "%s setrgbcolor %.3f setlinewidth ",
            kGcodeOriginMarkColor, size);
    fprintf(file_, "1 0 0 setrgbcolor %f %f %f moveto3d %f %f %f lineto3d "
            "stroke %% X\n", origin[AXIS_X], origin[AXIS_Y], origin[AXIS_Z],
            origin[AXIS_X] + 10*size, origin[AXIS_Y], origin[AXIS_Z]);
    fprintf(file_, "0 1 0 setrgbcolor %f %f %f moveto3d %f %f %f lineto3d "
            "stroke %% Y\n", origin[AXIS_X], origin[AXIS_Y], origin[AXIS_Z],
            origin[AXIS_X], origin[AXIS_Y] + 10*size, origin[AXIS_Z]);
    fprintf(file_, "0 0 1 setrgbcolor %f %f %f moveto3d %f %f %f lineto3d "
            "stroke %% Z\n", origin[AXIS_X], origin[AXIS_Y], origin[AXIS_Z],
            origin[AXIS_X], origin[AXIS_Y], origin[AXIS_Z] + 10*size);

    fprintf(file_, "grestore moveto3d\n");
  }

  void PrintPostscriptBoundingBox(float margin_x, float margin_y, float scale,
                                  float boundingbox_width) {
    // gs is notoriously bad in dealing with negative origin of the bounding
    // box, so move everything up with the translation box.
    const float range = 1.05 * GetDiagonalLength();
    if (boundingbox_width > 0) {
      scale = boundingbox_width / (range+margin_x);
    }
    const int bb_width = ToPoint(scale * (range+margin_x));
    const int bb_height = ToPoint(scale * (range+margin_y));
    fprintf(file_, "%%!PS-Adobe-3.0 EPSF-3.0\n"
            "%%%%BoundingBox: %d %d %d %d\n", 0, 0,
            bb_width, bb_height);
    fprintf(file_, "/per-page-setup {%.3f %.3f translate "
            "72 25.4 div dup scale %.3f dup scale} def\n",
            ToPoint(scale * (range+margin_x)/2),
            ToPoint(scale * (range+margin_y)/2), scale);
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

  static float ToPoint(float mm) {
    return roundf(mm / 25.4 * 72);
  }

  FILE *const file_;
  const AxesRegister machine_origin_;
  const VisualizationOptions opts_;

  float laser_intensity_ = 1.0;


  float last_laser_intensity_ = 1.0;
  bool last_move_rapid_ = false;
  bool just_homed_ = true;

  float laser_min_ = 1e6, laser_max_ = -1e6;
  AxesRegister min_;
  AxesRegister max_;
  ProcessingStep pass_ = ProcessingStep::Init;
  bool prefer_inch_ = false;
};

// Taking the low-level motor operations and visualize them. Uses color
// to visualize speed. Needs two passes - first pass determines available
// speed range, second outputs PostScript with colored segments.
// This also helps do determine if things line up properly with what the gcode
// parser spits out.
class MotorOperationsPrinter : public MotorOperations {
public:
  MotorOperationsPrinter(FILE *file, const MachineControlConfig &config,
                         float tool_dia, const VisualizationOptions &options)
    : file_(file), config_(config), opts_(options) {
    fprintf(file_, "\n%% -- Machine path. %s\n",
            opts_.show_speeds ? "Visualizing travel speeds" : "Simple.");
    fprintf(file_, "%% Note, larger tool diameters slow down "
            "PostScript interpreters a lot!\n");
    fprintf(file_, "/tool-diameter %.2f def\n\n", tool_dia);

    fprintf(file_, "tool-diameter setlinewidth\n");
    fprintf(file_, "0 0 0 moveto3d\n");
    if (!opts_.show_speeds) {
      fprintf(file_, "%s setrgbcolor\n", kMachineMoveColor);
    }
    fprintf(file_, "1 setlinejoin\n");

    if (opts_.output_js_vertices) {
      // TODO: this is wrong if the homing is on max.
      fprintf(stdout, "var machine_cube = [");
      fprintf(stdout, "[%.3f, %.3f, %.3f], [%.3f, %.3f, %.3f]];\n",
              0.0, 0.0, 0.0, config_.move_range_mm[AXIS_X],
              config_.move_range_mm[AXIS_Y], config_.move_range_mm[AXIS_Z]);
    }
  }

  ~MotorOperationsPrinter() {
    fprintf(file_, "stroke\n%% -- Finished Machine Path.\n");
  }

  void SetPass(ProcessingStep p) {
    pass_ = p;
    if (pass_ == ProcessingStep::GenerateOutput) {
      min_color_range_ = min_v_ + 0.1 * (max_v_ - min_v_);
      max_color_range_ = max_v_ - 0.1 * (max_v_ - min_v_);
#if 0
      fprintf(stderr, "Speed: [%.2f..%.2f]; Coloring span [%.2f..%.2f]\n",
              min_v_, max_v_, min_color_range_, max_color_range_);
#endif
      EmitMovetoPos();
    }
  }

  void RememberMinMax(float v) {
    if (v > max_v_) max_v_ = v;
    if (v < min_v_) min_v_ = v;
  }

  bool Enqueue(const LinearSegmentSteps &param) final {
    GCodeParserAxis dominant_axis = AXIS_X;
    for (int i = 1; i < BEAGLEG_NUM_MOTORS; ++i) {
      if (abs(param.steps[i]) > abs(param.steps[dominant_axis]))
        dominant_axis = (GCodeParserAxis) i;  // here, we have motor=axis
    }
    if (config_.steps_per_mm[dominant_axis] == 0 ||
        param.steps[dominant_axis] == 0) {
      return true;  // Nothing really to do.
    }

    switch (pass_) {
    case ProcessingStep::Init: assert(false); break;
    case ProcessingStep::Preprocess: {
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
    case ProcessingStep::GenerateOutput:
      PrintSegment(param, dominant_axis);
      break;
    }
    return true;
  }

  // Set the length of how long the smallest colored range should be.
  void SetColorSegmentLength(float length) {
    color_segment_length_ = length;
  }

  // Try to highlight if we are outside of coordinate system.
  bool inRange(int machine_pos, GCodeParserAxis axis) const {
    if (machine_pos < 0) return false;
    if (machine_pos > config_.move_range_mm[axis] * config_.steps_per_mm[axis])
      return false;
    return true;
  }

  bool isWithinMachineCube(const MotorsRegister &m) const {
    if (!opts_.show_out_of_range) return true;
    return inRange(m[AXIS_X], AXIS_X)
      && inRange(m[AXIS_Y], AXIS_Y)
      && inRange(m[AXIS_Z], AXIS_Z);
  }

  void PrintSegment(const LinearSegmentSteps &param,
                    GCodeParserAxis dominant_axis) {
    assert(pass_ == ProcessingStep::GenerateOutput);
    if (!param.steps[AXIS_X] && !param.steps[AXIS_Y] && !param.steps[AXIS_Z])
      return;
    MotorsRegister new_pos(current_pos_);
    new_pos[AXIS_X] += param.steps[AXIS_X];
    new_pos[AXIS_Y] += param.steps[AXIS_Y];
    new_pos[AXIS_Z] += param.steps[AXIS_Z];
    const bool is_valid_position = isWithinMachineCube(new_pos);
    const float dx_mm = param.steps[AXIS_X] / config_.steps_per_mm[AXIS_X];
    const float dy_mm = param.steps[AXIS_Y] / config_.steps_per_mm[AXIS_Y];
    const float dz_mm = param.steps[AXIS_Z] / config_.steps_per_mm[AXIS_Z];

    if (opts_.show_speeds) {
#if SHOW_SPEED_DETAILS
      fprintf(file_, "%% dx:%f dy:%f dz:%f %s (speed: %.1f->%.1f)\n",
              dx_mm, dy_mm, dz_mm,
              param.v0 == param.v1 ? "; steady move" :
              ((param.v0 < param.v1) ? "; accel" : "; decel"),
              param.v0/config_.steps_per_mm[dominant_axis],
              param.v1/config_.steps_per_mm[dominant_axis]);
#endif
      const float segment_len = sqrtf(dx_mm*dx_mm + dy_mm*dy_mm + dz_mm*dz_mm);
      if (segment_len == 0)
        return;  // If no movement in x/y/z (should've been caught earlier)
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

        const char *new_color = nullptr;
        if (is_valid_position) {
          int col_idx;
          if (v < min_color_range_) col_idx = 0;
          else if (v > max_color_range_) col_idx = 255;
          else {
            col_idx = roundf(255.0 * (v - min_color_range_)
                             / (max_color_range_-min_color_range_));
          }
          assert(col_idx >= 0 && col_idx < 256);
          if (col_idx != last_color_index_) {
            new_color = viridis_colors[col_idx];
            last_color_index_ = col_idx;
          }
        } else {
          new_color = kOutOfRangeColor;
        }
        if (new_color) {
          fprintf(file_, "%s switch-color ", new_color);
        }
#define TO_ABS_AXIS(a) (float(current_pos_[a]) + \
              float((i+1)*param.steps[a])/segments) / config_.steps_per_mm[a]

        fprintf(file_, "%.3f %.3f %.3f lineto3d",
                TO_ABS_AXIS(AXIS_X), TO_ABS_AXIS(AXIS_Y), TO_ABS_AXIS(AXIS_Z));
#undef TO_ABS_AXIS

        fprintf(file_, " %% %.1f mm/s [%s]", v,
                param.v0 == param.v1 ? "=" : (param.v0 < param.v1 ? "^" : "v"));
        if (i == 0)
          fprintf(file_, " (%d,%d,%d)",
                  param.steps[AXIS_X], param.steps[AXIS_Y], param.steps[AXIS_Z]);
        fprintf(file_, "\n");
      }
    } else {
      if (last_outside_machine_cube != is_valid_position) {
        // only print changes.
        fprintf(file_, "%s setrgbcolor ",
                is_valid_position ? kMachineMoveColor : kOutOfRangeColor);
        last_outside_machine_cube = is_valid_position;
      }
      fprintf(file_, "%f %f %f lineto3d %% %d %d %d\n",
              new_pos[AXIS_X] / config_.steps_per_mm[AXIS_X],
              new_pos[AXIS_Y] / config_.steps_per_mm[AXIS_Y],
              new_pos[AXIS_Z] / config_.steps_per_mm[AXIS_Z],
              new_pos[AXIS_X], new_pos[AXIS_Y], new_pos[AXIS_Z]);
    }

    current_pos_ = new_pos;
  }

  void MotorEnable(bool on) final {}
  void WaitQueueEmpty() final {}
  bool GetPhysicalStatus(PhysicalStatus *status) final { return false; }
  void SetExternalPosition(int motor, int pos) final {
    current_pos_[motor] = pos;
    if (pass_ == ProcessingStep::GenerateOutput) {
      EmitMovetoPos();
    }
  }

  void EmitMovetoPos() {
    fprintf(file_, "%f %f %f moveto3d  %% Homing one or more axes.\n",
            current_pos_[AXIS_X] / config_.steps_per_mm[AXIS_X],
            current_pos_[AXIS_Y] / config_.steps_per_mm[AXIS_Y],
            current_pos_[AXIS_Z] / config_.steps_per_mm[AXIS_Z]);
  }

  void PrintColorLegend(float x, float y, float width) {
    assert(pass_ == ProcessingStep::GenerateOutput);
    if (min_color_range_ >= max_color_range_)
      return;
    const float barheight = 0.05 * width;
    const float fontsize = barheight * 0.5;
    y -= barheight;
    fprintf(file_, "\n%% -- Color range visualization colorchart\n");
    fprintf(file_, "gsave /Helvetica findfont %.2f scalefont setfont\n",
            fontsize);
    fprintf(file_, "0 setgray 0 setlinewidth\n");
    fprintf(file_, "%.1f %.1f moveto (mm/s) show\n", x+width + 1, y-fontsize/2);
    fprintf(file_, "%.1f %.1f moveto 0 %.1f rlineto 0 0.5 rmoveto "
            "(<=%.1f) dup stringwidth pop 2 div neg 0 rmoveto show stroke\n",
            x, y, barheight, min_color_range_);

    const int kLegendPartitions = 6;
    for (int i = 1; i < kLegendPartitions; ++i) {
      float val = i * (max_color_range_-min_color_range_)/kLegendPartitions;
      float pos = i * width/kLegendPartitions;
      fprintf(file_, "%.1f %.1f moveto 0 %.1f rlineto 0 0.5 rmoveto "
              "(%.1f) dup stringwidth pop 2 div neg 0 rmoveto show stroke\n",
              x + pos, y, barheight, min_color_range_ + val);
    }
    fprintf(file_, "%.1f %.1f moveto 0 %.1f rlineto 0 0.5 rmoveto"
            "(>=%.1f) dup stringwidth pop 2 div neg 0 rmoveto show stroke\n",
            x+width, y, barheight, max_color_range_);

    fprintf(file_, "\n%% -- paint gradient\n");
    const float step = width/256;
    fprintf(file_, "%.3f setlinewidth %.1f %.1f moveto\n", barheight, x, y);
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
  const VisualizationOptions opts_;

  unsigned segment_count_ = 0;
  float min_v_ = 1e10;
  float max_v_ = -1e10;
  ProcessingStep pass_ = ProcessingStep::Init;
  float color_segment_length_ = 1;
  int last_color_index_ = -1;

  MotorsRegister current_pos_;
  float min_color_range_;   // These are set in pass==2
  float max_color_range_;

  bool last_outside_machine_cube = false;

  MotorOperationsPrinter(const MotorOperationsPrinter &);
};

static int usage(const char *progname, bool description = false) {
  std::string view_names;
  for (auto it : kNamedViews) {
    if (!view_names.empty()) view_names.append(", ");
    view_names.append(it.first);
  }

  if (description) {
    fprintf(stderr,
            "Utility to visualize the GCode path and the resulting machine\n"
            "movement with speed colorization, dependent on configuration.\n"
            "(Without config, only GCode path is shown)\n\n");
  }

  fprintf(stderr, "Usage: %s [options] <gcode-file>\n"
          "Options:\n"
          "\t-o <output-file>  : Name of output file; stdout default.\n"
          "\t-c <config>       : BeagleG machine config.\n"
          "\t-T <tool-diameter>: Tool diameter in mm.\n"
          "\t-t <threshold-angle>  : Threshold angle for accleration opt.\n"
          "\t-A <speed-tune-angle> : Speed tuning angle for accleration opt.\n"
          "\t-q                : Quiet.\n"
          "\t-s                : Visualize movement speeds.\n"
          "\t-D                : Don't show dimensions.\n"
          "\t-G                : Suppress GCode output, just show machine path.\n"
          "\t-M                : Suppress machine path output.\n"
          "\t-i                : Toggle show IJK control lines.\n"
          "\t-l                : [EXPERIMENTAL]: visualize laser intensity\n"
          "[---- Visualization ---- ]\n"
          "\t-w<width>         : Width in point (no unit) or mm (if appended)\n"
          "\t-e<distance>      : Eye distance in mm to show perspective.\n"
          "\t-a<frames>        : animation: create these number of frames "
          "showing rotation around vertical.\n"
          "\t-g<grid>          : Show grid on XY plane. Optional with 'in' unit suffix (default: mm).\n"
          "\t-C<caption>       : Caption text\n"
          "[---- Rotation. Multiple can be applied in sequence ----]\n"
          "\t-R<roll>          : Roll: Rotate around axis pointing towards and through canvas\n"
          "\t-P<roll>          : Pitch: Rotate around horizontal axis.\n"
          "\t-Y<roll>          : Yaw: Rotate around vertical axis.\n"
          "\t-V<view>          : Shortcut: view. One of {%s}\n",
          progname, view_names.c_str());
  return 1;
}

static void Reset(GCodeParser *parser, GCodePrintVisualizer *viz) {
  if (viz) viz->SetPass(ProcessingStep::Init);
  parser->ParseBlock(R"(
G90
G10 L2 P1 X0Y0Z0
G10 L2 P2 X0Y0Z0
G10 L2 P3 X0Y0Z0
G10 L2 P4 X0Y0Z0
G10 L2 P5 X0Y0Z0
G10 L2 P6 X0Y0Z0
G10 L2 P7 X0Y0Z0
G10 L2 P8 X0Y0Z0
G10 L2 P9 X0Y0Z0
G28
M02)", nullptr); // M02 needs to be last.
}

static bool ParseFile(GCodeParser *parser, const char *filename,
                      FILE *msg_stream) {
  if (!parser->ReadFile(fopen(filename, "r"), msg_stream)) {
    fprintf(stderr, "Cannot open %s\n", filename);
    return false;
  }

  return true;
}

static bool ParseConfigIfAvailable(const char *config_file,
                                   GCodeParser::Config *parser_cfg,
                                   MachineControlConfig *machine_config) {
  if (config_file) {
    ConfigParser config_parser;
    if (!config_parser.SetContentFromFile(config_file)) {
      fprintf(stderr, "Cannot read config file '%s'\n", config_file);
      return false;
    }
    if (!machine_config->ConfigureFromFile(&config_parser)) {
      fprintf(stderr, "Exiting. Parse error in configuration file '%s'\n",
              config_file);
      return false;
    }
  }

  // This is not connected to any machine. Don't assume homing, but
  // at least extract the home position from the config.
  machine_config->require_homing = false;
  for (const GCodeParserAxis axis : AllAxes()) {
    HardwareMapping::AxisTrigger trigger = machine_config->homing_trigger[axis];
    parser_cfg->machine_origin[axis] =
      (trigger & HardwareMapping::TRIGGER_MAX)
      ? machine_config->move_range_mm[axis]
      : 0;
  }
  return true;
}

// Append operation, making sure that the "arg" can properly parsed as float.
bool AppendFloatOp(const char *arg, const char *op,
                   std::vector<std::string> *result) {
  char *endptr = nullptr;
  strtof(arg, &endptr);
  if (*endptr != '\0') {
    fprintf(stderr, "Invalid floating point argument %s\n", arg);
    return false;
  }
  result->push_back(std::string(arg) + " " + op);
  return true;
}

bool AppendViewOp(const std::string &view, std::vector<std::string> *result) {
  const std::string key = ToLower(view);
  auto found = kNamedViews.lower_bound(key);
  if (found != kNamedViews.end() && HasPrefix(found->first, key)) {
    result->push_back(found->second);
    return true;
  } else {
    fprintf(stderr, "-V: Available views are valid prefix matches of ");
    for (auto it : kNamedViews) {
      fprintf(stderr, "'%s' ", it.first.c_str());
    }
    fprintf(stderr, "\n");
    return false;
  }
}

void ClearAndWarnIfNotEmpty(const char *msg, std::vector<std::string> *v) {
  if (!v->empty()) fprintf(stderr, "%s\n", msg);
  v->clear();
}

int main(int argc, char *argv[]) {
  const char *config_file = NULL;
  VisualizationOptions vis_options;
  FILE *output_file = stdout;
  std::string out_filename;
  float tool_diameter_mm = -1;
  bool show_dimensions = true;
  bool show_machine_path = true;  // Also requires config.
  bool show_gcode_path = true;
  float threshold_angle = 10;   // Same defaults as machine control.
  float speed_tune_angle = 60;
  bool range_check = false;
  float scale = 1.0f;
  float bounding_box_width_mm = -1;
  std::vector<std::string> rotate_ops;
  float eye_distance = -1;
  int animation_frames = -1;
  bool quiet = false;
  float grid = -1;

  int opt;
  while ((opt = getopt(argc, argv, "a:A:c:C:De:g:GilMo:P:qrR:sS:t:T:V:w:Y:")) != -1) {
    switch (opt) {
    case 'o':
      out_filename = optarg;
      output_file = fopen(out_filename.c_str(), "w");
      break;
    case 'q':
      quiet = true;
      break;
    case 't':
      threshold_angle = (float)atof(optarg);
      break;
    case 'A':
      speed_tune_angle = (float)atof(optarg);
      break;
    case 'g':
      grid = (float)atof(optarg);
      if (strstr(optarg, "in") != nullptr) grid *= 25.4;
      break;
    case 'l':
      vis_options.show_laser_burn = true;
      break;
    case 'C':
      vis_options.show_title = optarg;
      break;
    case 'c':
      config_file = strdup(optarg);
      break;
    case 'D':
      show_dimensions = false;
      break;
    case 'M':
      show_machine_path = false;
      break;
    case 'G':
      show_gcode_path = false;
      break;
    case 'T':
      tool_diameter_mm = atof(optarg);
      break;
    case 's':
      vis_options.show_speeds = true;
      break;
    case 'S':
      scale = atof(optarg);
      break;
    case 'w':
      bounding_box_width_mm = atof(optarg);
      if (strstr(optarg, "mm") == nullptr) {  // no mm ? Then back to pt.
        bounding_box_width_mm = bounding_box_width_mm / 72 * 25.4;
      }
      break;
    case 'e':
      eye_distance = atof(optarg);
      break;
    case 'i':
      vis_options.show_ijk = !vis_options.show_ijk;
      break;
    case 'r':
      range_check = true;
      break;
    case 'a':
      animation_frames = atoi(optarg);
      break;
    case 'R':
      if (!AppendFloatOp(optarg, "roll-rotate3d % -R option", &rotate_ops)) {
        return usage(argv[0]);
      }
      break;
    case 'P':
      if (!AppendFloatOp(optarg, "pitch-rotate3d % -P option", &rotate_ops)) {
        return usage(argv[0]);
      }
      break;
    case 'Y':
      if (!AppendFloatOp(optarg, "yaw-rotate3d % -Y option", &rotate_ops)) {
        return usage(argv[0]);
      }
      break;
    case 'V':
      ClearAndWarnIfNotEmpty("Chosen view resets previously applied rotations",
                             &rotate_ops);
      if (!AppendViewOp(optarg, &rotate_ops)) {
        return usage(argv[0]);
      }
      break;
    default:
      return usage(argv[0]);
    }
  }

  if (optind >= argc)
    return usage(argv[0], true);

  if (output_file == NULL) {
    fprintf(stderr, "Could not create output file.\n");
    return 1;
  }

  vis_options.show_speeds &= (show_machine_path);

  if (vis_options.show_speeds && !config_file) {
    fprintf(stderr, "FYI: Need machine-control config (-c) "
            "to show speeds (-s)\n");
    vis_options.show_speeds = false;
  }

  if (vis_options.show_laser_burn && !show_gcode_path) {
    // TODO: once we pass down the pwm in motor operations, that should work.
    fprintf(stderr, "FYI: Can only show laser color (-l) if GCode path is "
            "not disabled with -G\n");
    vis_options.show_laser_burn = false;
  }

  if (tool_diameter_mm >= 0 && !config_file) {
    fprintf(stderr, "FYI: Tool diameter (-T) only visualized "
            "if we have a machine config (-c)\n");
  }
  if (tool_diameter_mm < 0) tool_diameter_mm = 2.0f;  // some useful default

  const float printMargin = 2 + tool_diameter_mm/2;

  Log_init("/dev/null");

  // TODO(hzeller): only parse the file once, so that we can read from stdin.
  const char *filename = argv[optind];

  GCodeParser::Config parser_cfg;
  MachineControlConfig machine_config;
  if (!ParseConfigIfAvailable(config_file, &parser_cfg, &machine_config)) {
    return 1;
  }

  FILE *const msg_stream = quiet ? fopen("/dev/null", "w") : stderr;

  GCodeParser::Config::ParamMap parameters;  // TODO: read from file ?
  parser_cfg.parameters = &parameters;

  GCodePrintVisualizer gcode_printer(output_file, parser_cfg.machine_origin,
                                     machine_config.move_range_mm,
                                     vis_options);
  gcode_printer.SetPass(ProcessingStep::Preprocess);
  GCodeParser gcode_viz_parser(parser_cfg, &gcode_printer);
  ParseFile(&gcode_viz_parser, filename, msg_stream);
  Reset(&gcode_viz_parser, &gcode_printer);

  gcode_printer.PrintPostscriptBoundingBox(
    printMargin,
    printMargin + (vis_options.show_speeds ? 15 : 0),
    scale, bounding_box_width_mm);

  gcode_printer.PrintHeader(printMargin, eye_distance, animation_frames);

  if (animation_frames > 0) {
    // In case we do animations, we want to stuff all the postscript in one
    // function. However, they can be huge and create a stack-overflow while
    // reading. So we only enable it, when animation frames are requested.
    fprintf(output_file, "/show-stuff {\n");
  } else {
    fprintf(output_file, "per-page-setup\n");
  }

  if (!rotate_ops.empty()) {
    fprintf(output_file, "%% Rotation operations\n");
  }
  for (const std::string& op : rotate_ops) {
    fprintf(output_file, "%s\n", op.c_str());
  }

  gcode_printer.PrintModelFrame();
  if (grid > 0) gcode_printer.DrawGrid(grid);
  if (show_dimensions) gcode_printer.ShowMesaureLines();

  if (config_file && show_machine_path) {
    machine_config.threshold_angle = threshold_angle;
    machine_config.speed_tune_angle = speed_tune_angle;
    machine_config.acknowledge_lines = false;
    machine_config.range_check = range_check;

    // We never initialize the hardware mapping from the config file, so
    // we have a convenient mapping of gcode-axis == motor-number
    // Non-initialized hardware mapping behaves like initialized
    HardwareMapping hardware;

    MotorOperationsPrinter motor_operations_printer(
      output_file, machine_config, tool_diameter_mm, vis_options);
    GCodeMachineControl *machine_control
      = GCodeMachineControl::Create(machine_config, &motor_operations_printer,
                                    &hardware, nullptr, msg_stream);
    if (!machine_control) {
      // Ups, let's do it again with logging enabled to human-readably
      // print anything that might hint what the problem is.
      Log_init("/dev/stderr");
      Log_error("Cannot initialize machine:");
      GCodeMachineControl::Create(machine_config, &motor_operations_printer,
                                  &hardware, nullptr, stderr);
    } else {
      machine_control->SetMsgOut(NULL);
      motor_operations_printer.SetPass(ProcessingStep::Preprocess);
      GCodeParser parser(parser_cfg, machine_control->ParseEventReceiver());
      ParseFile(&parser, filename, msg_stream);
      Reset(&parser, nullptr);

      machine_control->SetMsgOut(msg_stream);
      // The moves are segmented into colored segments. Don't make segments
      // unnecessarily small but somehow relate it to the maximum size of the
      // result.
      motor_operations_printer.SetColorSegmentLength(
        gcode_printer.GetDiagonalLength()/100);
      motor_operations_printer.SetPass(ProcessingStep::GenerateOutput);
      ParseFile(&parser, filename, msg_stream);

      if (vis_options.show_speeds) {
        float x, y, w, h;
        gcode_printer.GetDimensions(&x, &y, &w, &h);
        motor_operations_printer.PrintColorLegend(x+0.05*w, y+h, 0.9*w);
      }
    }
    delete machine_control;
  }

  gcode_printer.ShowHomePos(parser_cfg.machine_origin);
  if (show_gcode_path) {
    // We print the gcode on top of the colored machine visualization.
    gcode_printer.SetPass(ProcessingStep::GenerateOutput);
    ParseFile(&gcode_viz_parser, filename, msg_stream);
  }

  if (animation_frames > 0) {
    fprintf(output_file, "} def\n");
    fprintf(output_file, "%s", R"(
/rot-angle 360.0 animation-frames div def
  0 1 animation-frames 1 sub {
  per-page-setup
  /matrix3d default-matrix3d def  % reset
  rot-angle mul roll-rotate3d     % Rotate around Z-axis
  show-stuff                      % Applies regular rotation and draws
  showpage
} for
)");
  } else {
    fprintf(output_file, "showpage\n");
  }

  fclose(output_file);

  if (!out_filename.empty() && !quiet) {
    const char *const f = out_filename.c_str();
    if (animation_frames > 0) {
      fprintf(stderr,
              "\n\n-- Convert the PostScript file to %s.gif animation with --\n"
              "gs -q -dBATCH -dNOPAUSE -sDEVICE=png16m "
              "-dGraphicsAlphaBits=4 -dTextAlphaBits=4 -dEPSCrop "
              "-sOutputFile=%s-anim-tmp-%%04d.png %s && "
              "convert %s-anim-tmp-????.png %s.gif && "
              //"ffmpeg -i %s-anim-tmp-%%04d.png -r 20 %s.mp4 && "
              "rm -f %s-anim-tmp-????.png\n", f, f, f, f, f, f);
    } else {
      fprintf(stderr,
              "\n\n-- Convert to image %s.png with --\n"
              "gs -q -dBATCH -dNOPAUSE -sDEVICE=png16m "
              "-dGraphicsAlphaBits=4 -dTextAlphaBits=4 -dEPSCrop "
              "-sOutputFile=%s.png %s\n", f, f, f);

    }
  }

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

const char *kPSHeader = R"(
% Internal 3D currentpoint register.
/last_x 0 def
/last_y 0 def
/last_z 0 def

/default-matrix3d [1 0 0  0 1 0  0 0 1] def

% 3D transformation matrix; modified by roll-..., pitch-... and yaw-rotate3d
/matrix3d default-matrix3d def

% Parameters two 3x3 matrix as array of 9
/dot-3 {
  3 dict begin
  /a exch def
  /b exch def
  /result [0 0 0  0 0 0  0 0 0] def
  /m { b exch get exch a exch get mul } def
  result 0   0 0 m  1 3 m  2 6 m add add put
  result 1   0 1 m  1 4 m  2 7 m add add put
  result 2   0 2 m  1 5 m  2 8 m add add put

  result 3   3 0 m  4 3 m  5 6 m add add put
  result 4   3 1 m  4 4 m  5 7 m add add put
  result 5   3 2 m  4 5 m  5 8 m add add put

  result 6   6 0 m  7 3 m  8 6 m add add put
  result 7   6 1 m  7 4 m  8 7 m add add put
  result 8   6 2 m  7 5 m  8 8 m add add put

  result
  end
} def

% Rotate around horizontal axis (pitch up and down)
% angle pitch-roate3d -
/pitch-rotate3d {
  1 dict begin
    /a exch def
    matrix3d [1 0 0  0  a cos  a sin neg   0  a sin  a cos] dot-3
  end
  /matrix3d exch def
} def

% Rotate around vertical axis.
% angle pitch-roate3d -
/yaw-rotate3d {
  1 dict begin
    /a exch def
    matrix3d [a cos  0  a sin   0 1 0   a sin neg 0 a cos] dot-3
  end
  /matrix3d exch def
} def

% Roll around axis pointing into the canvas
% angle yaw-roate3d -
/roll-rotate3d {
  1 dict begin
    /a exch def
    matrix3d [a cos  a sin neg  0  a sin  a cos 0  0 0 1] dot-3
  end
  /matrix3d exch def
} def

/project2d {
    4 dict begin
    center-z sub /z exch def
    center-y sub /y exch def
    center-x sub /x exch def
    % Super-simple projection just taking z-distance into account.
    perspective-eye-distance 0 gt {
      /pf perspective-eye-distance dup
          matrix3d 6 get x mul matrix3d 7 get y mul matrix3d 8 get z mul add add
          sub div def
    } {
      /pf 1 def
    } ifelse
    matrix3d 0 get x mul matrix3d 1 get y mul matrix3d 2 get z mul add add pf mul
    matrix3d 3 get x mul matrix3d 4 get y mul matrix3d 5 get z mul add add pf mul
    end
} def

% x y z moveto3d
/moveto3d {
    /last_z exch def
    /last_y exch def
    /last_x exch def
    last_x last_y last_z project2d moveto
 } def

/currentpoint3d {
  last_x last_y last_z
} def

/lineto3d {
  /last_z exch def
  /last_y exch def
  /last_x exch def
  last_x last_y last_z project2d lineto
 } def


% a lineto with stroke, but keeping currentpoint
/lineto3ds {
  /last_z exch def
  /last_y exch def
  /last_x exch def
  last_x last_y last_z project2d lineto
  currentpoint stroke moveto
 } def

/moveto3d-last {  % Useful after a stroke without having to push currentpoint3d
   last_x last_y last_z project2d moveto
} def

% r g b -
/switch-color {
   stroke setrgbcolor moveto3d-last
} def

% All lines should match smootly together.
1 setlinejoin

)";

}  // anonymous namespace
