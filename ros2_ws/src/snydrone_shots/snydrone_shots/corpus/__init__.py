"""Labelled shot-request corpus and evaluation harness for SNYdrone.

This package turns "I built a verifier" into a measured number. prompts.py
is a labelled corpus of structured shot specs, each carrying a ground-truth
label. evaluate.py runs every case through the real flight pipeline
(parse_shot_spec, sample_trajectory, check_trajectory), classifies the
system's decision, and scores it against the labels, isolating the
swept-path cases that a per-waypoint checker provably misses.
"""
