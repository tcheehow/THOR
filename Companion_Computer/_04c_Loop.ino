if (!sd.card()->writeStop()) {
  error("writeStop failed");
}
// Truncate file if recording stopped early.
if (bn != FILE_BLOCK_COUNT) {
  debugln(F("Truncating file"));
  if (!binFile.truncate(512L * bn)) {
    error("Can't truncate file");
  }
}
if (!binFile.rename(sd.vwd(), binName)) {
  error("Can't rename file");
}
debug(F("File renamed: "));
debugln(binName);
debug(F("Max block write usec: "));
debugln(maxLatency);
debug(F("Record time sec: "));
debugln(0.001 * (t1 - t0));
debug(minDelta);
debug(F(" <= jitter microseconds <= "));
debugln(maxDelta);
debug(F("Sample count: "));
debugln(count);
debug(F("Samples/sec: "));
debugln((1000.0)*count / (t1 - t0));
debug(F("Overruns: "));
debugln(overrunTotal);
debugln(F("Done"));

binaryToCsv();
WritingBlink();
}

// ===========================



