# Shimmer-C-API

# REV0_6
- fix to calibrated time stamp when using 3 byte raw time stamp (e.g. LogAndStream 0.6)

# REV0_5
- major updates to allow API to work with LogAndStream 0.6 and BtStream 0.8, 3 byte raw timestamp

# REV0_4
- minor update to packet loss detection, increasing the limit to 10%
- update to writesamplingrate makes sure internal sensor rates are approximately close/higher than shimmer sampling rate

# REV0_3_2
- Fix to filter, fix to to exg, gui failing when custom gain is used 
- Currently uses ShimmerClosedLibraryRev0_4

# REV0_3_1
- Minor fix to ppgtohr reset
- Currently uses ShimmerClosedLibraryRev0_4

# REV0_3
- Major update to ecgtohr algorithm and filtering algorithm, minor update to ppgtohr algorithm, user should see major improvements in both ecgtohr and ppgtohr algorithms.
- Currently uses ShimmerClosedLibraryRev0_3

