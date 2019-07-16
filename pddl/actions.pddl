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

(:durative-action get_order
  :parameters (?r - robot ?wp - waypoint)
  :duration ( = ?duration 10)
  :condition (and
    (at start(robot_at ?r ?wp))
 )
  :effect (and
    (at end(order_ready ?wp))
  )
)

(:durative-action set_order
  :parameters (?r - robot ?p - person ?wp - waypoint)
  :duration ( = ?duration 10)
  :condition (and
    (at start(wp_bar_location ?wp))
    (at start(barman ?p))
    (at start(robot_at ?r ?wp))
 )
  :effect (and
    (at end(order_to_barman ?r))
  )
)

(:durative-action check_order
  :parameters (?r - robot ?wp - waypoint)
  :duration ( = ?duration 10)
  :condition (and
    (at start(wp_bar_location ?wp))
    (at start(robot_at ?r ?wp))
 )
  :effect (and
    (at end(order_checked ?r))
  )
)

(:durative-action fix_order
  :parameters (?r - robot ?wp - waypoint)
  :duration ( = ?duration 10)
  :condition (and
    (at start(wp_bar_location ?wp))
    (at start(robot_at ?r ?wp))
 )
  :effect (and
    (at end(order_fixed ?r))
  )
)

(:durative-action deliver_order
  :parameters (?r - robot ?wp - waypoint)
  :duration ( = ?duration 10)
  :condition (and
    (at start(robot_at ?r ?wp))
 )
  :effect (and
    (at end(order_delivered ?wp))
  )
)
