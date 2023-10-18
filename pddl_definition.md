~~(:objects sugar_box spam_box)~~

~~(:predicates (drawer_open) (gripper_empty) (in_drawer ?arg1) (on_countertop ?arg1) (on_burner ?arg1))~~

(:types
    surface
    item
)

(:objects
    gripper
    drawer
    countertop burner - surface
    sugar_box spam_box - item
)

(:predicates
    (drawer_open)
    (in_drawer ?i - item)
    (drawer_empty)
    (gripper_empty)
    (on ?i - item ?s - surface)
    (clear ?s - surface)
)

(:action move ?from ?to) TODO

(:action pick_up
    :parameters (?i - item ?from - surface)
    :precondition (and (gripper_empty)
                       (clear ?from)
                       (on ?i ?from))
    :effect TODO
)

(:action pick_up_from_drawer
    :parameters (?i - item)
    :precondition (and (gripper_empty))
    :effect TODO)

(:action put_down
    :parameters (?i - item ?to - surface)
    :precondition (and (not (gripper_empty))
                       (clear ?to)
                       TODO item in gripper)
    :effect (and (not (clear ?to))
                 (on ?i ?to)
                 (not item in gripper)
                 (gripper_empty))
)

(:action open_drawer
    :precondition (and (gripper_empty)
                       (not (drawer_open)))
    :effect (not (drawer_open))
)

(:action close_drawer
    :precondition (and (gripper_empty)
                       (drawer_open))
    :effect (not (drawer_open))
)

(:init
    (on sugar_box burner)
    (on spam_box countertop)
    (gripper_empty)
    (drawer_empty) ?
    (drawer_open) ?
)

(:goal
    (on sugar_box countertop)
    (in_drawer spam_box)
    (not (drawer_open))
)