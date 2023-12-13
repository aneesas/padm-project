; DOMAIN FILE

(define (domain kitchen)
    (:requirements :strips :typing :negative-preconditions)

    (:types
        drawer 
        gripper
        surface
        item
    )

    (:predicates
        (open_drawer)
        (clear_drawer)
        (clear_gripper)
        (clear_countertop)
        (clear_burner)
        (burner_has_sugarbox)
        (burner_has_spambox)
        (countertop_has_sugarbox)
        (countertop_has_spambox)
        (drawer_has_sugarbox)
        (drawer_has_spambox)
        (gripper_has_sugarbox)
        (gripper_has_spambox)
        (at_countertop)
        (at_burner)
        (at_drawer)
    )

    ; MOVE -----------------------------------------
    (:action moveDtoB
        :parameters ()
        :precondition (at_drawer)
        :effect (and (at_burner)
                (not (at_countertop))
                (not (at_drawer))
        )
    )

    (:action moveBtoD
        :parameters ()
        :precondition (at_burner)
        :effect (and (at_drawer)
                (not (at_countertop))
                (not (at_burner))
        )
    )

    (:action moveDtoC
        :parameters ()
        :precondition (at_drawer)
        :effect (and (at_countertop)
                (not (at_drawer))
                (not (at_burner))
        )
    )

    (:action moveCtoD
        :parameters ()
        :precondition (at_countertop)
        :effect (and (at_drawer)
                (not (at_countertop))
                (not (at_burner))
        )
    )

    (:action moveCtoB
        :parameters ()
        :precondition (at_countertop)
        :effect (and (at_burner)
                (not (at_countertop))
                (not (at_drawer))
        )
    )

    (:action moveBtoC
        :parameters ()
        :precondition (at_burner)
        :effect (and (at_countertop)
                (not (at_drawer))
                (not (at_burner))
        )
    )


    ; PICK UP --------------------------------------

    (:action pick_up_spamAtD
        :parameters ()
        :precondition(and
            (clear_gripper)
            (drawer_has_spambox)
            (at_drawer)
            (open_drawer)
        )
        :effect (and
            (not (clear_gripper))
            (gripper_has_spambox)
            (not (drawer_has_spambox))
            (clear_drawer)
        )
    )

    (:action pick_up_spamAtB
        :parameters ()
        :precondition(and
            (clear_gripper)
            (burner_has_spambox)
            (at_burner)
        )
        :effect (and
            (not (clear_gripper))
            (gripper_has_spambox)
            (not (burner_has_spambox))
            (clear_burner)
        )
    )
     
    (:action pick_up_spamAtC
        :parameters ()
        :precondition(and
            (clear_gripper)
            (countertop_has_spambox)
            (at_countertop)
        )
        :effect (and
            (not (clear_gripper))
            (gripper_has_spambox)
            (not (countertop_has_spambox))
            (clear_countertop)
        )
    )

    (:action pick_up_sugarAtD
        :parameters ()
        :precondition(and
            (clear_gripper)
            (drawer_has_sugarbox)
            (at_drawer)
            (open_drawer)
        )
        :effect (and
            (not (clear_gripper))
            (gripper_has_sugarbox)
            (not (drawer_has_sugarbox))
            (clear_drawer)
        )
    )

    (:action pick_up_sugarAtB
        :parameters ()
        :precondition(and
            (clear_gripper)
            (burner_has_sugarbox)
            (at_burner)
        )
        :effect (and
            (not (clear_gripper))
            (gripper_has_sugarbox)
            (not (burner_has_sugarbox))
            (clear_burner)
        )
    )
     
    (:action pick_up_sugarAtC
        :parameters ()
        :precondition(and
            (clear_gripper)
            (countertop_has_sugarbox)
            (at_countertop)
        )
        :effect (and
            (not (clear_gripper))
            (gripper_has_sugarbox)
            (not (countertop_has_sugarbox))
            (clear_countertop)
        )
    )

    ; PUT DOWN -------------------------------------

    (:action put_down_spamAtD
        :parameters ()
        :precondition(and
            (clear_drawer)
            (gripper_has_spambox)
            (open_drawer)
            (at_drawer)
        )
        :effect (and
            (not (clear_drawer))
            (drawer_has_spambox)
            (not (gripper_has_spambox))
            (clear_gripper)
        )
    )

    (:action put_down_spamAtB
        :parameters ()
        :precondition(and
            (clear_burner)
            (gripper_has_spambox)
            (at_burner)
        )
        :effect (and
            (not (clear_burner))
            (burner_has_spambox)
            (not (gripper_has_spambox))
            (clear_gripper)
        )
    )
        
    (:action put_down_spamAtC
        :parameters ()
        :precondition(and
            (clear_countertop)
            (gripper_has_spambox)
            (at_countertop)
        )
        :effect (and
            (not (clear_countertop))
            (countertop_has_spambox)
            (not (gripper_has_spambox))
            (clear_gripper)
        )
    )

    (:action put_down_sugarAtD
        :parameters ()
        :precondition(and
            (clear_drawer)
            (gripper_has_sugarbox)
            (open_drawer)
            (at_drawer)
        )
        :effect (and
            (not (clear_drawer))
            (drawer_has_sugarbox)
            (not (gripper_has_sugarbox))
            (clear_gripper)
        )
    )

    (:action put_down_sugarAtB
        :parameters ()
        :precondition(and
            (clear_burner)
            (gripper_has_sugarbox)
            (at_burner)
        )
        :effect (and
            (not (clear_burner))
            (burner_has_sugarbox)
            (not (gripper_has_sugarbox))
            (clear_gripper)
        )
    )
        
    (:action put_down_sugarAtC
        :parameters ()
        :precondition(and
            (clear_countertop)
            (gripper_has_sugarbox)
            (at_countertop)
        )
        :effect (and
            (not (clear_countertop))
            (countertop_has_sugarbox)
            (not (gripper_has_sugarbox))
            (clear_gripper)
        )
    )


    ; OPEN/CLOSER DRAWER ---------------------------

    (:action open_drawer
        :parameters ()
        :precondition (and
            (not (open_drawer))
            (clear_gripper)
            (at_drawer)
        )
        :effect (open_drawer)
    )

    (:action close_drawer
        :parameters ()
        :precondition (and
            (open_drawer)
            (clear_gripper)
            (at_drawer)
        )
        :effect (not(open_drawer))
    )
)