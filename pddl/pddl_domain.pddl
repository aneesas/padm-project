; DOMAIN FILE

(define (domain kitchen)
    (:requirements :strips :typing :negative-preconditions)

    (:types
        drawer gripper - container
        surface container - location
        item
    )

    (:predicates
        (open ?c - container)
        (clear ?c - location)
        (contains ?i - item ?l - location)
        (at ?g - gripper ?l - location)
    )

    (:action move
        :parameters (?from - location ?to - location ?g - gripper)
        :precondition (at ?g ?from)
        :effect (at ?g ?to)
    )

    (:action pick_up
        :parameters (?i - item ?from - location ?g - gripper)
        :precondition(and
            (clear ?g)
            (contains ?i ?from)
        )
        :effect (and
            (not (clear ?g))
            (contains ?i ?g)
            (not (contains ?i ?from))
            (clear ?from)
        )
    )

    (:action put_down
        :parameters (?i - item ?to - location ?g - gripper)
        :precondition(and
            (clear ?to)
            (contains ?i ?g)
        )
        :effect (and
            (not (clear ?to))
            (contains ?i ?to)
            (not (contains ?i ?g))
            (clear ?g)
        )
    )

    (:action pick_up_drawer
        :parameters (?i - item ?from - drawer ?g - gripper)
        :precondition(and
            (clear ?g)
            (contains ?i ?from)
            (open ?from)
        )
        :effect (and
            (not (clear ?g))
            (contains ?i ?g)
            (not (contains ?i ?from))
            (clear ?from)
        )
    )

    (:action put_down_drawer
        :parameters (?i - item ?to - drawer ?g - gripper)
        :precondition(and
            (clear ?to)
            (contains ?i ?g)
            (open ?to)
        )
        :effect (and
            (not (clear ?to))
            (contains ?i ?to)
            (not (contains ?i ?g))
            (clear ?g)
        )
    )

    (:action open_drawer
        :parameters (?d - drawer ?g - gripper)
        :precondition (and
            (not (open ?d))
            (clear ?g)
        )
        :effect (open ?d)
    )

    (:action close_drawer
        :parameters (?d - drawer ?g - gripper)
        :precondition (and
            (open ?d)
            (clear ?g)
        )
        :effect (not(open ?d))
    )
)