This shows what are the parameters need to be changed in warp.

Need to modify the warp omnigraph file to add more attributes:

Find 
{ISAACSIM ROOT FOLDER}/IsaacSim-4.5.0/extscache/omni.warp-1.5.0/omni/warp/nodes/_impl/OgnParticlesSimulate.py

Setup the normal direction
```
# Register the ground.
builder.set_ground_plane(
    offset=-db.inputs.groundAltitude,
    normal=[0.0, 0.0, 1.0],  # Add this line to setup the ground normal direction
    ke=db.inputs.contactElasticStiffness * db.inputs.globalScale,
    kd=db.inputs.contactDampingStiffness * db.inputs.globalScale,
    kf=db.inputs.contactFrictionStiffness * db.inputs.globalScale,
    mu=db.inputs.contactFrictionCoeff,
)
```