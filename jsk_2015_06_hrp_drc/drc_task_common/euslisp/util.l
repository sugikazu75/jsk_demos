;; utility function

(defun call-empty-service (srv)
  (let ((req (instance std_srvs::EmptyRequest :init)))
    (ros::service-call srv req)))

;; topic <---> compressed message conversion

(defun coords-to-float-vector (coords length)
  ;6 parameters are filled
  (let ((ret (instantiate float-vector length))) 
    (setf (elt ret 0) (elt (send coords :worldpos) 0))
    (setf (elt ret 1) (elt (send coords :worldpos) 1))
    (setf (elt ret 2) (elt (send coords :worldpos) 2))
    (setf (elt ret 3) (elt (car (send coords :rpy-angle)) 0))
    (setf (elt ret 4) (elt (car (send coords :rpy-angle)) 1))
    (setf (elt ret 5) (elt (car (send coords :rpy-angle)) 2))
    ret))

(defun float-vector-to-coords (fv)
  (make-coords :pos (subseq fv 0 3)
               :rpy (subseq fv 3 6)))

(defun pos-to-float-vector (pos length)
  (let ((ret (instantiate float-vector length)))
    (setf (elt ret 0) (elt pos 0))
    (setf (elt ret 1) (elt pos 1))
    (setf (elt ret 1) (elt pos 1))
    ret))

(defun rect-polygon-stamped-to-integer-vector (msg length)
  (let ((point-a (list (send (elt (send msg :polygon :points) 0) :x)
                       (send (elt (send msg :polygon :points) 0) :y)))
        (point-b (list (send (elt (send msg :polygon :points) 1) :x)
                       (send (elt (send msg :polygon :points) 1) :y))))
    (coerce (append (list (floor (elt point-a 0))
                          (floor (elt point-a 1))
                          (floor (elt point-b 0))
                          (floor (elt point-b 1)))
                    (make-list (- length 4) :initial-element 0))
            integer-vector)))

(defun integer-vector-to-rect-polygon-stamped (msg)
  (let ((x1 (elt (send msg :image_roi_info) 0))
        (y1 (elt (send msg :image_roi_info) 1))
        (x2 (elt (send msg :image_roi_info) 2))
        (y2 (elt (send msg :image_roi_info) 3)))
    (let ((ret (instance geometry_msgs::PolygonStamped :init)))
      (send ret :header :stamp (ros::time-now))
      (let ((p1 (instance geometry_msgs::Point32 :init))
            (p2 (instance geometry_msgs::Point32 :init)))
        (send p1 :x x1)
        (send p1 :y y1)
        (send p2 :x x2) 
        (send p2 :y y2)
        (send ret :polygon :points (list p1 p2))
        ret))))

;; It's a circle actually
(defun torus-to-float-vector (coords radius length)
  (let ((ret (coords-to-float-vector coords length))) ;6 parameters are filled
    (setf (elt ret 6) radius)
    ret))

(defun float-vector-to-torus (params)
  (let ((coords (float-vector-to-coords params))
        (radius (elt params 6)))
    (list coords radius)))

;; message handler
(defclass narrowband-message-handler
  :super propertied-object
  :slots (deligate-map))

(defmethod narrowband-message-handler
  (:init ()
    (setq deligate-map nil)
    self)
  (:root-callback (msg)
    (let ((type (send msg :type)))
      (print (assoc type deligate-map))
      (if (assoc type deligate-map)
          (send (car (cdr (assoc type deligate-map)))
                (cdr (cdr (assoc type deligate-map)))
                msg)
        (ros::ros-warn "received unknown type ~A" type))))
  (:register-callback (type deligate-object deligate-method)
    (if (assoc type deligate-map)
        (error "you've already registered ~A" type))
    (setq deligate-map (append
                        (list (cons type
                                    (cons deligate-object
                                          deligate-method)))
                        deligate-map))
    )
  (:register-callbacks (&rest specs)
    (dolist (spec specs)
      (ros::ros-warn "registering ~A" spec)
      (send* self :register-callback spec)))
  )
