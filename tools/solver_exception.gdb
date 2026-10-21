set pagination off
set breakpoint pending on
python
import gdb
class SolverThrow(gdb.Breakpoint):
    def stop(self):
        frame = gdb.newest_frame()
        for _ in range(12):
            if frame is None:
                break
            name = frame.name() or ""
            library = gdb.solib_name(frame.pc()) or ""
            if "gtsam" in name or "libgtsam" in library:
                return True
            frame = frame.older()
        return False
SolverThrow("__cxa_throw")
end
run
bt 25
frame 1
info args
