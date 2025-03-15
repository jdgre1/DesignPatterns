# bug_zapper/views.py
from rest_framework import viewsets
from .models import MovingBug
from .serializers import MovingBugSerializer

class MovingBugViewSet(viewsets.ModelViewSet):
    queryset = MovingBug.objects.all()
    serializer_class = MovingBugSerializer