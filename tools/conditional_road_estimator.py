#!/usr/bin/env python3
"""Independent offline D2 process; fixed true Anchors, never vehicle truth."""
from road_estimator import main

if __name__ == '__main__':
    main(conditional_oracle=True)
