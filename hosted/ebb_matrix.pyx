cdef extern from "inttypes.h":
     ctypedef unsigned int uint32_t

cdef extern from "ebbrt/EbbId.h" namespace "ebbrt":
     ctypedef uint32_t EbbId

cdef extern from "ebbrt/EbbRef.h" namespace "ebbrt":
     cdef cppclass EbbRef[T]:
          EbbRef()
          EbbRef(EbbId id)
          T* GetPointer()
          EbbId operator()

cdef extern from "ebbrt/Future.h" namespace "ebbrt":
     cdef cppclass Future[T]:
          pass

cdef extern from "Matrix.h":
     cdef cppclass Matrix:
          Future[double] Get(size_t, size_t)

cdef extern from "Matrix.h" namespace "Matrix":
     Future[EbbRef[Matrix]] Create(size_t)

cdef extern from "ebb_matrix_helper.h":
     void activate_context()
     void deactivate_context()
     EbbRef[Matrix] wait_for_future_ebbref_matrix "wait_for_future<ebbrt::EbbRef<Matrix>>" (Future[EbbRef[Matrix]]*) except +
     double wait_for_future_double "wait_for_future<double>"(Future[double]*) except +

cdef class EbbMatrix:
     cdef EbbRef[Matrix] matrix
     cdef int size
     def __cinit__(self, size):
         activate_context()
         cdef Future[EbbRef[Matrix]] fut = Create(size)
         self.matrix = wait_for_future_ebbref_matrix(&fut)
         deactivate_context()
     def __init__(self, size):
         self.size = size
     def __getitem__(self, pos):
         row,column = pos
         activate_context()
         cdef Matrix* ref = self.matrix.GetPointer()
         cdef Future[double] fut = ref.Get(row, column)
         cdef double ret = wait_for_future_double(&fut)
         deactivate_context()
         return ret
