(:durative-action check_table_status
  :parameters (?r - robot ?wp - waypoint ?t - table)
  :duration ( = ?duration 10)
  :condition (and
    (at start(robot_at ?r ?wp))
    (at start(is_wp_near_table ?wp ?t))
)
  :effect (and
    (at end(table_checked ?t))
  )
)

(:durative-action get_order
  :parameters (?r - robot ?wp - waypoint ?t - table)
  :duration ( = ?duration 10)
  :condition (and
    (at start(robot_at ?r ?wp))
    (at start(is_wp_near_table ?wp ?t))
 )
  :effect (and
    (at end(order_ready ?t))
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
  :parameters (?r - robot ?wp - waypoint ?t - table)
  :duration ( = ?duration 10)
  :condition (and
    (at start(robot_at ?r ?wp))
    (at start(is_wp_near_table ?wp ?t))
 )
  :effect (and
    (at end(order_delivered ?t))
  )
)

;(:durative-action gretting_new_customer
;  :parameters (?r - robot ?p - person ?wp - waypoint ?t - table)
;  :duration ( = ?duration 10)
;  :condition (and
;    (at start(person_guided ?p ?wp))
;    (at start(is_wp_near_table ?wp ?t))
; )
;  :effect (and
;    (at end(new_customer_attended ?p ?t))
;  )
;)


(:durative-action check_waiting_person
  :parameters (?r - robot ?p - person ?wp - waypoint ?z - zone)
  :duration ( = ?duration 10)
  :condition (and
    (at start(robot_at ?r ?wp))
    (at start(person_at ?p ?wp))
    (at start(wp_in_zone ?wp ?z))
 )
  :effect (and
    (at end(person_detected ?r ?p ?z))
  )
)

(:durative-action attend_person
  :parameters (?r - robot ?p - person ?wp - waypoint ?t - table ?z - zone)
  :duration ( = ?duration 10)
  :condition (and
    (at start(person_detected ?r ?p ?z))
    (at start(person_guided ?p ?wp))
    (at start(is_wp_near_table ?wp ?t))
 )
  :effect (and
    (at end(attended_person ?p ?t))
  )
)
