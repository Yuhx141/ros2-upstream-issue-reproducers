The documented URDF-file order still does not hold on current supported releases.

I tested a URDF that declares `z_joint` before `a_joint`, with distinct state interfaces and values. Both Jazzy 4.42.1 (`aacd8426`) and 6.9.0 (`78d6508e`) publish/request the order `[a_joint, z_joint]` in 3/3 independent processes. The explicit-parameter positive control `joints: [z_joint, a_joint]` preserves `[z_joint, a_joint]` in 3/3 on both versions.

The remaining URDF path iterates `urdf::Model::joints_`, whose declared type is `std::map<std::string, JointSharedPtr>`, so it yields lexical name order rather than declaration order. The current user documentation says the order is the same as the order in the URDF file.

This appears to be the same root family as #159 and an incomplete edge of #1572, so I am adding evidence here rather than opening a duplicate. I can provide the small gtest patch if useful.
