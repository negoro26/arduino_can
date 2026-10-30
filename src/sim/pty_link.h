#ifndef PTY_LINK_H
#define PTY_LINK_H

#include "link_mux.h"

class PtyLink final : public ByteLink {
 public:
  explicit PtyLink(const char *path) : path_(path) {}

  // Opens the PTY in raw mode and points `path` at its slave. false (with a
  // message on stderr) on failure.
  bool open();
  int fd() const { return master_; }

  uint16_t read(uint8_t *out, uint16_t capacity) override;
  void write(const char *bytes, uint16_t length) override;

 private:
  const char *path_;
  int master_ = -1;
  // Held open for the simulator's lifetime: while no slave fd is open, reads
  // on the master fail with EIO, and clients open and close the port often.
  int slave_ = -1;
};

#endif  // PTY_LINK_H
