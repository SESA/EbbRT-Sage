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
          Future[void] Set(size_t, size_t, double)
          Future[EbbRef[Matrix]] Multiply(EbbRef[Matrix])
          Future[void] Randomize()
          Future[double] Sum()
          Future[void] AllocNodes()
          void Destroy()

cdef extern from "LocalMatrix.h":
     cdef cppclass LocalMatrix:
          Future[void] Randomize()
          void Destroy()

cdef extern from "Matrix.h" namespace "Matrix":
     Future[EbbRef[Matrix]] Create(size_t, size_t, size_t, size_t)

cdef extern from "LocalMatrix.h" namespace "LocalMatrix":
     Future[EbbRef[LocalMatrix]] CreateLocal(size_t, size_t)

cdef extern from "ebb_matrix_helper.h":
     void activate_context()
     void deactivate_context()
     EbbRef[Matrix] wait_for_future_ebbref_matrix "wait_for_future<ebbrt::EbbRef<Matrix>>" (Future[EbbRef[Matrix]]*) except +
     EbbRef[LocalMatrix] wait_for_future_ebbref_localmatrix "wait_for_future<ebbrt::EbbRef<LocalMatrix>>" (Future[EbbRef[LocalMatrix]]*) except +
     double wait_for_future_double "wait_for_future<double>"(Future[double]*) except +
     void wait_for_future_void "wait_for_future<void>"(Future[void]*) except +

cdef class EbbMatrix:
     cdef EbbRef[Matrix] matrix

     cdef setMatrix(self, EbbRef[Matrix] matrix):
          self.matrix = matrix
          return self

     @staticmethod
     def create(x_dim, y_dim, x_tile, y_tile):
         activate_context()
         cdef Future[EbbRef[Matrix]] fut = Create(x_dim, y_dim, x_tile, y_tile)
         matrix = wait_for_future_ebbref_matrix(&fut)
         deactivate_context()
         return EbbMatrix().setMatrix(matrix)

     def __getitem__(self, pos):
         row,column = pos
         activate_context()
         cdef Matrix* ref = self.matrix.GetPointer()
         cdef Future[double] fut = ref.Get(row, column)
         cdef double ret = wait_for_future_double(&fut)
         deactivate_context()
         return ret
     def __setitem__(self, pos, value):
         row,column = pos
         activate_context()
         cdef Matrix* ref = self.matrix.GetPointer()
         cdef Future[void] fut = ref.Set(row, column, value)
         wait_for_future_void(&fut)
         deactivate_context()
     def __mul__(self, EbbMatrix other):
         cdef EbbMatrix e = self
         activate_context()
         cdef Matrix* ref = e.matrix.GetPointer()
         cdef Future[EbbRef[Matrix]] fut = ref.Multiply(other.matrix)
         matrix = wait_for_future_ebbref_matrix(&fut)
         deactivate_context()
         return EbbMatrix().setMatrix(matrix)
     def randomize(self):
         cdef EbbMatrix e = self
         activate_context()
         cdef Matrix* ref = e.matrix.GetPointer()
         cdef Future[void] fut = ref.Randomize()
         wait_for_future_void(&fut)
         deactivate_context()
     def sum(self):
         cdef EbbMatrix e = self
         activate_context()
         cdef Matrix* ref = e.matrix.GetPointer()
         cdef Future[double] fut = ref.Sum()
         cdef double ret = wait_for_future_double(&fut)
         deactivate_context()
         return ret
     def AllocNodes(self):
         cdef EbbMatrix e = self
         activate_context()
         cdef Matrix* ref = e.matrix.GetPointer()
         cdef Future[void] fut = ref.AllocNodes()
         wait_for_future_void(&fut)
         deactivate_context()
     def __dealloc__(self):
         cdef EbbMatrix e = self
         activate_context()
         cdef Matrix* ref = e.matrix.GetPointer()
         ref.Destroy()
         deactivate_context();

cdef class LocalEbbMatrix:
     cdef EbbRef[LocalMatrix] matrix

     cdef setMatrix(self, EbbRef[LocalMatrix] matrix):
          self.matrix = matrix
          return self

     @staticmethod
     def create(x_dim, y_dim):
         activate_context()
         cdef Future[EbbRef[LocalMatrix]] fut = CreateLocal(x_dim, y_dim)
         matrix = wait_for_future_ebbref_localmatrix(&fut)
         deactivate_context()
         return LocalEbbMatrix().setMatrix(matrix)
        
     def randomize(self):
         cdef LocalEbbMatrix e = self
         activate_context()
         cdef LocalMatrix* ref = e.matrix.GetPointer()
         cdef Future[void] fut = ref.Randomize()
         wait_for_future_void(&fut)
         deactivate_context()

     def __dealloc__(self):
         cdef LocalEbbMatrix e = self
         activate_context()
         cdef LocalMatrix* ref = e.matrix.GetPointer()
         ref.Destroy()
         deactivate_context();
