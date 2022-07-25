#!/bin/bash
rsync -av --filter "merge .rsync" ./ racecar:~/Projects/cps_racecar