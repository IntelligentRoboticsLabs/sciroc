(:durative-action check_table_status
  :parameters (?r - robot ?wp - waypoint)
  :duration ( = ?duration 10)
  :condition (and
    (at start(robot_at ?r ?wp))
 )
  :effect (and
    (at end(table_checked ?wp))
  )
)
