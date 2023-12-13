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
        (burner_has_sugarbox)
        (countertop_has_spambox)
        (clear_gripper)
        (clear_drawer)
        (open_drawer)
        (at_drawer)
    )

    (:goal (and
        (countertop_has_sugarbox)
        (drawer_has_spambox)
        (not (open_drawer))
        )
    )
)