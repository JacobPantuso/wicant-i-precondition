# Local walk-away locking

WiCAN owns execution; automatic requests never fall back to Bluelink.
The matching app is required: XAL appends a sixth field `1`, indicating mandatory OBD-off arming.
`ATXALP1` records on and disarms; `ATXALP0` authorizes off after an on session.
CAN silence does not arm. READY cancels pending departure.
`ATXALB1` holds until `ATXALB0`; missing readings never clear a hold.

No approach detection or automatic unlock code is present on this fix branch.
Approach work is isolated on `feature/wican-approach-unlock`.
Default configuration disables sleep; installed NVS may differ. Sleep policy is unchanged.
