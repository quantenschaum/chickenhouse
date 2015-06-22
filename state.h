// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// created by Adam, chickenhouse@louisenhof2.de, 2015
// https://github.com/quantenschaum/chickenhouse

class State {
  public:
    int state;
    boolean latch;
    unsigned long flank;
    State(int s) {
      state = s;
      latch = true;
      flank = millis();
    }
    int get() {
      return state;
    }
    boolean is(int v) {
      return state == v;
    }
    void set(int s) {
      if (s != state) {
        state = s;
        latch = true;
        flank = millis();
      }
    }
    boolean changed() {
      boolean l = latch;
      latch = false;
      return l;
    }
    unsigned long time() {
      return flank;
    }
    unsigned long age() {
      return millis() - flank;
    }
};
