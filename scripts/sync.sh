#!/bin/bash
rsync -av --filter "merge .rsyncignore" ./ racecar:~/Projects/cps_racecar