document.addEventListener('DOMContentLoaded', () => {
  document.querySelectorAll('.quiz').forEach(q => {
    const choices = q.querySelectorAll('button.choice');
    choices.forEach(btn => btn.addEventListener('click', () => {
      if (q.classList.contains('answered')) return; // one try
      // mark selection
      const correct = btn.dataset.correct === 'true';
      btn.classList.add(correct ? 'correct' : 'wrong');
      // show the correct one if user was wrong
      if (!correct) q.querySelector('button.choice[data-correct="true"]')?.classList.add('correct');
      // lock and reveal feedback
      q.classList.add('answered');
      choices.forEach(b => b.disabled = true);
      // optional: custom feedback text
      const fb = q.querySelector('.feedback');
      if (fb && !fb.dataset.init) {
        fb.innerHTML = correct ? (fb.dataset.ok || "✅ Correct!") : (fb.dataset.no || "❌ Not quite.");
        fb.dataset.init = '1';
      }
    }));
  });
});
