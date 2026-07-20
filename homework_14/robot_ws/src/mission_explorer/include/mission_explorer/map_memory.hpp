#pragma once

#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <utility>

namespace explorer {

using Coord = std::pair<int, int>;  // (x, y)

// What we know a cell to be. We fold the world's string cell_type
// ("#", ".", "S", "C", "x") into a small enum for fast comparisons.
enum class CellKind {
  Unknown,  // never seen
  Wall,     // "#"
  Floor,    // "." or "S" or "x" — walkable
  Contact,  // "C" — walkable-adjacent target, still active
};

class MapMemory {
public:
  // Record what a single observed cell is. Called once per cell per scan.
  void observe(const Coord& at, CellKind kind) { known_[at] = kind; }

  // Mark that the robot has physically stood on this cell.
  void mark_visited(const Coord& at) { visited_.insert(at); }

  // Is this cell known AND walkable (something we could legally step onto)?
  // Floor and Contact-turned-floor are walkable; Wall and Unknown are not.
  // BLANK 3: return true only if `at` is in known_ and its kind is walkable.
  bool is_walkable(const Coord& at) const
  {
    auto it = known_.find(at);
    if (it == known_.end()) {
      return false;  // unknown cells are not walkable
    }
    return it->second == CellKind::Floor || it->second == CellKind::Contact;
  }

  // Have we already stood on this cell?
  bool is_visited(const Coord& at) const { return visited_.count(at) > 0; }

  // Is this cell known at all (seen in any scan)?
  bool is_known(const Coord& at) const { return known_.count(at) > 0; }

private:
  std::map<Coord, CellKind> known_;  // every cell we've observed
  std::set<Coord> visited_;          // cells we've stood on
};

}  // namespace explorer