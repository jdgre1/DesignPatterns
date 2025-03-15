# bug_zapper_api/urls.py
from django.contrib import admin
from django.urls import path, include
from rest_framework.routers import DefaultRouter
from bug_zapper.views import MovingBugViewSet  # Vergewissere dich, dass der Import stimmt

router = DefaultRouter()
router.register(r'bugs', MovingBugViewSet)

urlpatterns = [
    path('admin/', admin.site.urls),
    path('api/', include(router.urls)),  # Der API-Pfad
]
