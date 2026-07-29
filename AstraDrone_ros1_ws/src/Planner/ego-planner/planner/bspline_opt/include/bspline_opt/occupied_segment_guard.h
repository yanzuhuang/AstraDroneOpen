#ifndef BSPLINE_OPT_OCCUPIED_SEGMENT_GUARD_H_
#define BSPLINE_OPT_OCCUPIED_SEGMENT_GUARD_H_

namespace ego_planner
{

inline bool isValidOccupiedSegment(int in_id, int out_id)
{
  return in_id >= 0 && out_id > in_id;
}

}  // namespace ego_planner

#endif  // BSPLINE_OPT_OCCUPIED_SEGMENT_GUARD_H_
