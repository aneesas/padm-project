;PROBLEM FILE

(define (problem activity_planning)
    (:domain kitchen)
    (:requirements :strips :typing)

    (:objects
        gpr - gripper
        dwr - drawer
        countertop burner - surface
        sugar_box spam_box - item 
    )

    (:init
        (contains sugar_box burner)
        (contains spam_box countertop)
        (clear gpr)
        (clear dwr)
        (open dwr)
    )

    (:goal (and
        (contains sugar_box countertop)
        (contains spam_box dwr)
        (not (open dwr))
        )
    )
)